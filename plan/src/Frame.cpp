#include <planning/Frame.h>

// Status Frame::Init(const PlanningConf &planning_conf)
// {
//     forward_distance_limit = planning_conf.planning_param.frame_forward_distance_limit;
//     forward_time = planning_conf.planning_param.frame_forward_time;
//     safe_distance = planning_conf.planning_param.frame_safe_distance;

//     status.status = "OK";
//     return status;
// }
void Frame::Init(const PlanningConf &planning_conf)
{
    forward_distance_limit = planning_conf.planning_param.frame_forward_distance_limit;
    forward_time = planning_conf.planning_param.frame_forward_time;
    safe_distance = planning_conf.planning_param.frame_safe_distance;
}
void Frame::GetObstacleRange(map<int,Obstacle> &obstacle_list)
{
    if(effective_obstacles.empty())
    {
        ROS_INFO("there is no effective obstacle");
        return;
    }
    for(int i=0;i<effective_obstacles.size();++i)
    {
        (*(obstacle_list.find(effective_obstacles[i]))).second.is_avoided=true;
        Obstacle ob_temp = (*(obstacle_list.find(effective_obstacles[i]))).second;
        TransformToFrenet(ob_temp,ob_temp.frenet_pos);
        min_s = min(min_s,ob_temp.frenet_pos.s-ob_temp.ObstacleBox.diagonal/2);
        max_s = max(max_s,ob_temp.frenet_pos.s+ob_temp.ObstacleBox.diagonal/2);
        min_d = min(min_d,ob_temp.frenet_pos.d-ob_temp.ObstacleBox.diagonal/2);
        max_d = max(max_d,ob_temp.frenet_pos.d+ob_temp.ObstacleBox.diagonal/2);
        ROS_INFO_STREAM("min_s "<<min_s);
        ROS_INFO_STREAM("max_s "<<max_s);
        ROS_INFO_STREAM("min_d "<<min_d);
        ROS_INFO_STREAM("max_d "<<max_d);
    }
    
}
void Frame::TransformToFrenet(Obstacle &obstacle, FrenetPathPoint & frenet_point)
{
    ROS_INFO("Obstacle TransformToFrenet");
    double x = obstacle.GetX();
    double y = obstacle.GetY();
    ROS_INFO_STREAM("ob actual x is "<<x);
    ROS_INFO_STREAM("ob actual y is "<<y);
    int id = MatchPoint(x,y);
    prius_msgs::My_Trajectory_Point point = reference_line_frenet.trajectory_points[id];
    ROS_INFO_STREAM("ob goal x is "<<point.x);
    ROS_INFO_STREAM("ob goal y is "<<point.y);  
    double dx = point.x-x;
    double dy = point.y-y;   
    double point_distance = sqrt(dx*dx+dy*dy);
    double alpha = atan2(dy,dx);
    double angle = PI - alpha + point.theta;
    ROS_INFO_STREAM("goal s is "<<point.s);
    ROS_INFO_STREAM("point_distance*cos(angle) is "<<point_distance*cos(angle));
    ROS_INFO_STREAM("origin_point.s "<<origin_point.s);
    frenet_point.s = point.s + point_distance*cos(angle)-origin_point.s;
    frenet_point.s_d = obstacle.GetSpeed()*sin(obstacle.GetHeading() -point.theta);
    frenet_point.s_dd = 0;
    frenet_point.d = point_distance*sin(angle);
    frenet_point.d_d = obstacle.GetSpeed()*cos(obstacle.GetHeading() - point.theta);
    frenet_point.d_dd = 0;
    ROS_INFO_STREAM("ob frenet_point.s "<<frenet_point.s);
    ROS_INFO_STREAM("ob frenet_point.s_d "<<frenet_point.s_d);
    ROS_INFO_STREAM("ob frenet_point.s_dd "<<frenet_point.s_dd);
    ROS_INFO_STREAM("ob frenet_point.d "<<frenet_point.d);  
    ROS_INFO_STREAM("ob frenet_point.d_d "<<frenet_point.d_d);
    ROS_INFO_STREAM("ob frenet_point.d_dd "<<frenet_point.d_dd);
}
void Frame::TransformToFrenet(VehicleState &vehicle_state, FrenetPathPoint & frenet_point)
{
    ROS_INFO("TransformToFrenet");
    double x = vehicle_state.movement_state.pose.position.x;
    double y = vehicle_state.movement_state.pose.position.y;
    ROS_INFO_STREAM("actual x is "<<x);
    ROS_INFO_STREAM("actual y is "<<y);
    int id = MatchPoint(x,y);
    prius_msgs::My_Trajectory_Point point = reference_line_frenet.trajectory_points[id];
    ROS_INFO_STREAM("goal x is "<<point.x);
    ROS_INFO_STREAM("goal y is "<<point.y);
    ROS_INFO_STREAM("goal s is "<<point.s);
    // double dx = point.x-x;
    // double dy = point.y-y;        
    // double point_distance = sqrt(dx*dx+dy*dy);
    // double alpha = atan2(dy,dx);
    // double angle = PI - alpha + point.theta;
    double dx = x-point.x;
    double dy = y-point.y;        
    double point_distance = sqrt(dx*dx+dy*dy);
    double alpha = atan2(dy,dx);
    double angle = alpha - point.theta;
    
    ROS_INFO_STREAM("point_distance*cos(angle) is "<<point_distance*cos(angle));
    ROS_INFO_STREAM("origin_point.s "<<origin_point.s);
    double cos_delta_theta = cos(vehicle_state.heading_angle -point.theta);
    double sin_delta_theta = sin(vehicle_state.heading_angle -point.theta);
    frenet_point.s = point.s + point_distance*cos(angle)-origin_point.s;
    frenet_point.s_d = vehicle_state.current_velocity*cos_delta_theta;
    frenet_point.s_dd = vehicle_state.acceleration*cos_delta_theta;
    frenet_point.d = point_distance*sin(angle);
    frenet_point.d_d = vehicle_state.current_velocity*sin_delta_theta/cos_delta_theta;
    frenet_point.d_dd = vehicle_state.acceleration*sin_delta_theta/(cos_delta_theta*cos_delta_theta*vehicle_state.current_velocity);
    // double dx = x-point.x;
    // double dy = y-point.y; 
    // double cross_rd_nd = dy*cos(point.theta)-dx*sin(point.theta);
    
    // double point_distance = sqrt(dx*dx+dy*dy);
    // frenet_point.d = copysign(point_distance,cross_rd_nd);
    // double delta_theta = vehicle_state.heading_angle-point.theta;
    // double one_minus_kappa_r_d = 1 - point.kappa * frenet_point.d;
    // frenet_point.d_d = one_minus_kappa_r_d * tan(delta_theta);
    // const double kappa_r_d_prime =
    //     point.dkappa * frenet_point.d + point.kappa * frenet_point.d_d;

    // frenet_point.d_dd =-kappa_r_d_prime*tan(delta_theta)+one_minus_kappa_r_d/cos(delta_theta)/cos(delta_theta)*(point.kappa * one_minus_kappa_r_d/cos(delta_theta)-point.kappa);

    // frenet_point.s = point.s-origin_point.s;
    // frenet_point.s_d = vehicle_state.current_velocity*cos(delta_theta)/one_minus_kappa_r_d;
    // const double delta_theta_prime =
    //     one_minus_kappa_r_d / cos(delta_theta) * point.kappa - point.kappa;

    // frenet_point.s_dd = (vehicle_state.acceleration* cos(delta_theta) -frenet_point.s_d * frenet_point.s_d*(frenet_point.d_d*delta_theta_prime-kappa_r_d_prime))/one_minus_kappa_r_d;

    ROS_INFO_STREAM("frenet_point.s "<<frenet_point.s);
    ROS_INFO_STREAM("frenet_point.s_d "<<frenet_point.s_d);
    ROS_INFO_STREAM("frenet_point.s_dd "<<frenet_point.s_dd);
    ROS_INFO_STREAM("frenet_point.d "<<frenet_point.d);  
    ROS_INFO_STREAM("frenet_point.d_d "<<frenet_point.d_d);
    ROS_INFO_STREAM("frenet_point.d_dd "<<frenet_point.d_dd);  

}

double Frame::NormalizeAngle(double angle)
{
    double a = fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) 
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI; 
}



int Frame::MatchPoint(double x,double y)
{
    vector<float> dist;
    for(int i=0;i<reference_line_frenet.trajectory_points.size();++i)
    {
        float point_distance;
        prius_msgs::My_Trajectory_Point point = reference_line_frenet.trajectory_points[i];
        double dx = point.x-x;
        double dy = point.y-y;
        point_distance = sqrt(dx*dx+dy*dy);
        dist.push_back(point_distance);
    }
    vector<float>::iterator goal=min_element(dist.begin(), dist.end());
    return distance(dist.begin(), goal);
}
void Frame::Update(int fid, double x, double y,double time_stamp,VehicleState &v_state)
{
    frame_id = fid;
    start_time=time_stamp;
    start_x = x;
    start_y = y;
    vehicle_state = v_state;
}

void Frame::GetReferenceLine(GlobalTrajectory &global_trajectory)
{
    reference_line_frenet.trajectory_points.clear();
    double forward_distance = max(forward_distance_limit, 1.5*vehicle_state.current_velocity * forward_time);
    ROS_INFO_STREAM("forward_distance is "<<forward_distance);
    int point_id = global_trajectory.goal_id;
    double init_s = global_trajectory.trajectory_info[point_id].s;
    ++point_id;
    
    double distance =0.0;
    while(distance<2*forward_distance&&point_id<global_trajectory.trajectory_info.size())
    {
        prius_msgs::My_Trajectory_Point point_a = global_trajectory.trajectory_info[point_id];
        reference_line_frenet.trajectory_points.push_back(point_a);
        
        distance = global_trajectory.trajectory_info[point_id].s - init_s;
        ++point_id;
    }
    origin_point = reference_line_frenet.trajectory_points.front();
}
void Frame::AddObstacle(int obstacle_id)
{
    effective_obstacles.push_back(obstacle_id);
}
bool Frame::HaveObstacle()
{
    if(effective_obstacles.empty())
    {
        return false;
    }
    else
    {
        return true;
    }
    
}
bool Frame::CheckCollision(Obstacle &obstacle)
{
    if(obstacle.is_avoided)
    {
        return true;
    }
    vector<prius_msgs::My_Trajectory_Point> reference_line_info = reference_line_frenet.trajectory_points;
    for(int i=(reference_line_info.size())*2/3;i>=0;--i)
    {
        prius_msgs::My_Trajectory_Point point = reference_line_info[i];
        double distance = obstacle.ComputeDistance(point.x,point.y);
        if(distance<(obstacle.ObstacleBox.diagonal+vehicle_state.VehicleBox.diagonal)/2+safe_distance)
        {
            return false;
        }
    }
    return true;
}
int Frame::MatchPointInFrenet(FrenetPathPoint &point)
{
    double frenet_point_s = point.s+origin_point.s;
    for(int i = frenet_goal_id;i<reference_line_frenet.trajectory_points.size();++i)
    {
        if(frenet_point_s-reference_line_frenet.trajectory_points[i].s<0)
        {
            double forward_distance = frenet_point_s-reference_line_frenet.trajectory_points[i].s;
            double backward_distance = frenet_point_s-reference_line_frenet.trajectory_points[i-1].s;
            if(-forward_distance<backward_distance)
            {
                frenet_goal_id = i;
                return i;
            }
            else
            {
                frenet_goal_id = i-1;
                return i-1;
            }
        }
    }
    return -1;
}
void Frame::TransformToCartesian()
{
    ROS_INFO("TransformToCartesian");
    FrenetPath tmp_path = best_trajectory_frenet.frenet_path;
    int id;
    for(int i=0;i<tmp_path.FrenectPoints.size();++i)
    {
        FrenetPathPoint frenet_point = tmp_path.FrenectPoints[i];
        ROS_INFO_STREAM("frenet point i = "<<i);
        ROS_INFO_STREAM("frenet point s = "<<frenet_point.s);
        ROS_INFO_STREAM("frenet point d = "<<frenet_point.d);
        id = MatchPointInFrenet(frenet_point);
        // if(i%20==0)
        // {
        //     ROS_INFO_STREAM("ID = "<<id);
        // }
        //ROS_INFO_STREAM("ID = "<<id);
        prius_msgs::My_Trajectory_Point goal_point = reference_line_frenet.trajectory_points[id];
        prius_msgs::My_Trajectory_Point tmp_point;

        tmp_point.x = goal_point.x+frenet_point.d*cos(goal_point.theta+PI/2);
        tmp_point.y = goal_point.y+frenet_point.d*sin(goal_point.theta+PI/2);
        
        tmp_point.dkappa=0;
        tmp_point.gear=1;
        tmp_point.kappa;
        tmp_point.relative_time = frenet_point.t;
        tmp_point.s = frenet_point.s;
        if(i==0)
        {
            tmp_point.theta = 0;
        }
        else
        {
            tmp_point.theta = atan2(tmp_point.y-last_y,tmp_point.x-last_x);
        }
        
        //tmp_point.v = sqrt(frenet_point.d_d*frenet_point.d_d+frenet_point.s_d*frenet_point.s_d)
        tmp_point.v = 15;
        tmp_point.a = 0;
        tmp_point.z = 1;




        // Apollo 方法
        // const double cos_ref_theta = cos(goal_point.theta);
        // const double sin_ref_theta = sin(goal_point.theta);

        // tmp_point.x = goal_point.x -sin_ref_theta*frenet_point.d;
        // tmp_point.y = goal_point.y -cos_ref_theta*frenet_point.d;

        // const double one_minus_kappa_r_d = 1 - goal_point.kappa * frenet_point.d;
        // const double tan_delta_theta = frenet_point.d_d / one_minus_kappa_r_d;
        // const double delta_theta = std::atan2(frenet_point.d_d, one_minus_kappa_r_d);
        // const double cos_delta_theta = std::cos(delta_theta);

        // tmp_point.theta = NormalizeAngle(delta_theta+goal_point.theta);
        // const double kappa_r_d_prime =goal_point.dkappa * frenet_point.d + goal_point.kappa * frenet_point.d_d;
        // const double d_dot = frenet_point.d_d * frenet_point.s_d;              
        // tmp_point.kappa = (((frenet_point.d_dd + kappa_r_d_prime * tan_delta_theta) *cos_delta_theta * cos_delta_theta) /(one_minus_kappa_r_d) +goal_point.kappa) *cos_delta_theta / (one_minus_kappa_r_d);
        // tmp_point.v = sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *frenet_point.s_d * frenet_point.s_d +d_dot * d_dot);
        // const double delta_theta_prime =one_minus_kappa_r_d / cos_delta_theta * tmp_point.kappa - goal_point.kappa;
        // tmp_point.a = frenet_point.s_dd * one_minus_kappa_r_d / cos_delta_theta +frenet_point.s_d * frenet_point.s_d / cos_delta_theta *(frenet_point.d_d * delta_theta_prime - kappa_r_d_prime);
        
        
        // tmp_point.v = 15;
        // tmp_point.a = 0;
        ROS_INFO("==========================");
        ROS_INFO_STREAM("goal_point.x is "<<goal_point.x);
        ROS_INFO_STREAM("goal_point.y is "<<goal_point.y);
        //ROS_INFO_STREAM("L is "<<L);
        //ROS_INFO_STREAM("alpha is "<<alpha);
        ROS_INFO_STREAM("theta is "<<goal_point.theta);
        //ROS_INFO_STREAM("cos(beta) is "<<cos(beta));
        //ROS_INFO_STREAM("sin(beta) is "<<sin(beta));
        ROS_INFO_STREAM("x = "<<tmp_point.x);
        ROS_INFO_STREAM("y = "<<tmp_point.y);
        ROS_INFO_STREAM("z = "<<tmp_point.z);
        ROS_INFO_STREAM("v = "<<tmp_point.v);
        ROS_INFO_STREAM("a = "<<tmp_point.a);
        ROS_INFO_STREAM("theta = "<<tmp_point.theta);
        ROS_INFO_STREAM("s = "<<tmp_point.s);
        ROS_INFO_STREAM("t = "<<tmp_point.relative_time);
        ROS_INFO_STREAM("kappa = "<<tmp_point.kappa);
        ROS_INFO_STREAM("gear = "<<tmp_point.gear);
        ROS_INFO("==========================");
        // if(i%100==0)
        // {
        //     ROS_INFO("==========================");
        //     ROS_INFO_STREAM("goal_point.x is "<<goal_point.x);
        //     ROS_INFO_STREAM("goal_point.y is "<<goal_point.y);
        //     //ROS_INFO_STREAM("L is "<<L);
        //     //ROS_INFO_STREAM("alpha is "<<alpha);
        //     ROS_INFO_STREAM("theta is "<<goal_point.theta);
        //     //ROS_INFO_STREAM("cos(beta) is "<<cos(beta));
        //     //ROS_INFO_STREAM("sin(beta) is "<<sin(beta));
        //     ROS_INFO_STREAM("x = "<<tmp_point.x);
        //     ROS_INFO_STREAM("y = "<<tmp_point.y);
        //     ROS_INFO_STREAM("z = "<<tmp_point.z);
        //     ROS_INFO_STREAM("v = "<<tmp_point.v);
        //     ROS_INFO_STREAM("a = "<<tmp_point.a);
        //     ROS_INFO_STREAM("theta = "<<tmp_point.theta);
        //     ROS_INFO_STREAM("s = "<<tmp_point.s);
        //     ROS_INFO_STREAM("t = "<<tmp_point.relative_time);
        //     ROS_INFO_STREAM("kappa = "<<tmp_point.kappa);
        //     ROS_INFO_STREAM("gear = "<<tmp_point.gear);
        //     ROS_INFO("==========================");
        // }
        best_trajectory.trajectory_points.push_back(tmp_point);
    }
}