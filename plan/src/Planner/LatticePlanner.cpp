#include <planning/Planner/LatticePlanner.h>

Status LatticePlanner::Init(const PlanningConf &planning_conf)
{
    target_velocity = planning_conf.planning_param.target_velocity;
    lateral_sample_num = planning_conf.planning_param.lateral_sample_num;
    lateral_sample_interval=planning_conf.planning_param.lateral_sample_interval;
    station_sample_num=planning_conf.planning_param.station_sample_num;
    station_sample_interval = planning_conf.planning_param.station_sample_interval;
    time_sample_num = planning_conf.planning_param.time_sample_num;
    time_sample_interval = planning_conf.planning_param.time_sample_interval;
    status.status = "OK";
    return status;
}
Status LatticePlanner::PlanOnReferenceLine(VehicleState &vehicle_state, Frame &frame,prius_msgs::My_Trajectory &local_trajectory)
{
    
    frame.TransformToFrenet(vehicle_state,frame.start_point);
    frame.start_point.t = 0;
    CreateEndState(vehicle_state,frame);
    ROS_INFO_STREAM("the num of trajectories to be planned is "<<frame.end_point_set.size());
    for(int i=0;i<frame.end_point_set.size();++i)
    {
        ROS_INFO_STREAM("end   point s is "<<frame.end_point_set[i].s);
        ROS_INFO_STREAM("end   point d is "<<frame.end_point_set[i].d);
        ROS_INFO_STREAM("end   point t is "<<frame.end_point_set[i].t);
        polynomials polys;
        ROS_INFO_STREAM("the "<<i<<" trajectory is being generating");
        trajectory_generator.GenerateTrajectory(frame.start_point,frame.end_point_set[i],polys);
        polys.frenet_path.cost = trajectory_generator.ComputeCost(polys.frenet_path,target_velocity);
        CollisionCheck(polys,frame,vehicle_state);
        if(polys.frenet_path.is_collided)
        {
            ROS_INFO_STREAM("the "<<i<<" trajectory will collide obstacle");
        }
        if(!polys.frenet_path.is_valid)
        {
            ROS_INFO_STREAM("the "<<i<<" trajectory is unvalid");
        }
        if(!polys.frenet_path.is_collided)
        {
            frame.polynomial_set.push_back(polys);
        }
    }
    ROS_INFO_STREAM("The num of safe trajectory is "<<frame.polynomial_set.size());
    GetBestTrajectory(frame);
    local_trajectory = frame.best_trajectory;
    // 碰撞检查
    // 找到最小cost
    // 转变回笛卡尔坐标系

    status.status = "OK";
    return status;
}
void LatticePlanner::CollisionCheck(polynomials &polys,Frame &frame,VehicleState &vehicle_state)
{
    ROS_INFO("CollisionCheck");
    double min_s = frame.min_s;
    double max_s = frame.max_s;
    double min_d = frame.min_d;
    double max_d = frame.max_d;
    double vechile_diagonal = 1.2*vehicle_state.VehicleBox.diagonal;
    for(int i =0;i<polys.frenet_path.FrenectPoints.size();++i)
    {
        if(!polys.frenet_path.is_valid)
        {
            continue;
        }
        FrenetPathPoint point = polys.frenet_path.FrenectPoints[i];
        if(point.s<max_s+vechile_diagonal&&point.s>min_s-vechile_diagonal&&point.d<max_d+vechile_diagonal&&point.d>min_d-vechile_diagonal)
        {
            polys.frenet_path.is_collided=true;
            return;
        }
    }
    ROS_INFO("no trajectory will collide obstacle");

}
void LatticePlanner::GetBestTrajectory(Frame &frame)
{
    ROS_INFO("GetBestTrajectory");
    double min_cost_id = 0;
    for(int i=1;i<frame.polynomial_set.size();++i)
    {
        if(!frame.polynomial_set[i].frenet_path.is_valid||frame.polynomial_set[i].frenet_path.is_collided)
        {
            continue;
        }
        if(frame.polynomial_set[i].frenet_path.cost<frame.polynomial_set[min_cost_id].frenet_path.cost)
        {
            min_cost_id = i;
        }
    }
    frame.best_trajectory_frenet = frame.polynomial_set[min_cost_id];
    ROS_INFO_STREAM("the min cost id = "<<min_cost_id);
    frame.TransformToCartesian();
}


void LatticePlanner::CreateEndState(VehicleState &vehicle_state,Frame &frame)
{
    ROS_INFO("CreateEndState");
    FrenetPathPoint end_point;
    end_point.d_d=0;
    end_point.d_dd=0;
    end_point.s_d=target_velocity;
    end_point.s_dd=0;
    if(vehicle_state.is_off_track||vehicle_state.is_going_back)
    {
        
        double forward_time = 3.5;
        double s_sample_num = 3;
        double t_sample_num = 3;
        double s_sample_interval = 2;
        double t_sample_interval = 10;
        double forward_distance = vehicle_state.current_velocity*forward_time;
        double init_s = frame.reference_line_frenet.trajectory_points[0].s;
        // int target_id;
        // for(int i =0;i<frame.reference_line_frenet.trajectory_points.size();++i)
        // {
        //     if((frame.reference_line_frenet.trajectory_points[i].s-init_s)>forward_distance)
        //     {
        //         target_id = i;
        //         return;
        //     }
        // }
        // double target_s = frame.reference_line_frenet.trajectory_points[target_id].s-init_s;
        end_point.d = 0;
        for(int i=0;i<s_sample_num;++i)
        {
            end_point.s = 2*forward_distance-(s_sample_num/2)*s_sample_interval+i*s_sample_interval;
            for(int j = 0;j<t_sample_num;++j)
            {
                end_point.t = floor(forward_time)*(1-floor(t_sample_num/2)*t_sample_interval/100+j*t_sample_interval/100);
                ROS_INFO("===============================");
                ROS_INFO_STREAM("end point s is "<<end_point.s);
                ROS_INFO_STREAM("end point v is "<<end_point.s_d);
                ROS_INFO_STREAM("end point d is "<<end_point.d);
                ROS_INFO_STREAM("end point t is "<<end_point.t);
                ROS_INFO("===============================");
                frame.end_point_set.push_back(end_point);
            }
        }

    }
    else
    {
        double start_d = frame.start_point.d;
        double obstacle_d = (frame.max_d+frame.min_d)/2;
        double approximate_time = (frame.min_s-frame.start_point.s)/vehicle_state.current_velocity;
        ROS_INFO_STREAM("approximate_time is "<<approximate_time);

        for(int i=0;i<lateral_sample_num;++i)
        {
            if(start_d>obstacle_d)
            {
                end_point.d = frame.max_d + (i+1)*lateral_sample_interval;
            }
            else
            {
                end_point.d = frame.min_d -(i+1)*lateral_sample_interval;
            }
            for(int j =0;j<station_sample_num;++j)
            {
                end_point.s = frame.max_s+(j+4)*station_sample_interval;

                for(int q=0;q<time_sample_num;++q)
                {
                    end_point.t = floor(approximate_time)*(1-floor(time_sample_num/2)*time_sample_interval/100+q*time_sample_interval/100);
                    ROS_INFO("===============================");
                    ROS_INFO_STREAM("end point s is "<<end_point.s);
                    ROS_INFO_STREAM("end point v is "<<end_point.s_d);
                    ROS_INFO_STREAM("end point d is "<<end_point.d);
                    ROS_INFO_STREAM("end point t is "<<end_point.t);
                    ROS_INFO("===============================");
                    frame.end_point_set.push_back(end_point);
                }
            }
        }
    }
    


}

