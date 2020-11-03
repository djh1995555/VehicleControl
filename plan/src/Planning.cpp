#include<planning/Planning.h>

int Planning::Spin()
{
    Status init_status = Init();
    if(init_status.status != "OK")
    {
        ROS_INFO("Planning Init Failed!");
        return -1;
    }

    Status start_status = Start();
    if(start_status.status != "OK")
    {
        ROS_INFO("Planning Start Failed!");
        return -2;
    }

    ros::spin();
    return 0;
}
Status Planning::Init()
{
    init_time = (ros::Time::now()).toSec();
    ROS_INFO("Planning Init...");
    ROS_INFO("Planning ...");
    Status read_status = planning_conf.ReadControlConf(PLAN_CONF_FILE_NAME);
    // ROS_INFO_STREAM("Ts is "<<planning_conf.planning_param.Ts);
    // ROS_INFO_STREAM("frwheel_steering_d_gain is "<<planning_conf.planning_param.frwheel_steering_d_gain);
    if(read_status.status != "OK")
    {
        ROS_INFO_STREAM("Unable to load plan conf file "<<PLAN_CONF_FILE_NAME);
        status.status = "Failed";
        return status;
    }
    ROS_INFO_STREAM("Planning conf file:"<<PLAN_CONF_FILE_NAME<<" had beed loaded!");
    
    Status vehicle_instance_status = vehicle_state.Init(planning_conf);
    if(vehicle_instance_status.status != "OK")
    {
        ROS_INFO("Vehicle state init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }
    ROS_INFO("Vehicle state init successfully!");

    Status routing_status = global_trajectory.Init(planning_conf);
    if(routing_status.status != "OK")
    {
        ROS_INFO("Global trajectory init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }   
    ROS_INFO("Global trajectory init successfully!");

    Status agent_status = planner_agent.Init(planning_conf);
    if(agent_status.status != "OK")
    {
        ROS_INFO("Planner agent init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }
    ROS_INFO("Planner agent init successfully!");

    // Status frame_status = frame.Init(planning_conf);
    // if(frame_status.status != "OK")
    // {
    //     ROS_INFO("Frame init failed! Stopping...");
    //     status.status = "Failed";
    //     return status;       
    // }
    // ROS_INFO("Frame init successfully!");
    status.status = "OK";
    return status;
}
Status Planning::Start()
{
    ROS_INFO("Planning start after sleeping for 1000ms...");
    ros::Duration(1).sleep();
    vehicle_state.state = "WORKING!";
    ROS_INFO_STREAM("Vehicle current state is "<<vehicle_state.state);

    ontime_trajectory_pub=node.advertise<prius_msgs::My_Trajectory>("/ontime_trajectory",10);
    ROS_INFO("ontime_trajectory_pub");
    obstacle_pub = node.advertise<prius_msgs::Obstacles>("/obstacle_info",10);
    global_trajectory_sub=node.subscribe("/global_trajectory",10,&Planning::TrajectoryCallback,this);
    ROS_INFO("global_trajectory_sub");
    localization_sub=node.subscribe("/base_pose_ground_truth",10,&Planning::LocalPlanning,this);
    ROS_INFO("localization_sub");
    
    status.status = "OK";
    return status;
}
void Planning::Stop()
{
    // TO DO
}

void Planning::TrajectoryCallback(const prius_msgs::My_Trajectory &routing)
{
    ROS_INFO("read trajectory");
    Status trajectory_status =global_trajectory.ReadTrajectory(routing);
    if(trajectory_status.status != "OK")
    {
        ROS_INFO("Read trajectory failed!");
        return;
    }
    global_trajectory.PrintTrajectory();
    ROS_INFO("read path");
    if(planner_agent.activate_planning_function)
    {
        CreateObstacles(planning_conf.planning_param.obstacles_num);
    }
    
}
bool Planning::IsOnTrajectory(const double x,const double y)
{
    // int i =local_goal_id;
    // int min_distance_pos=local_goal_id;
    // double min_distance = 99999;
    // while(i<local_goal_id+local_search_length&&i<last_published_trajectory.trajectory_points.size())
    // {
    //     double point_distance;
    //     prius_msgs::My_Trajectory_Point point = last_published_trajectory.trajectory_points[i];
    //     point_distance = sqrt((point.x-x)*(point.x-x)+(point.y-y)*(point.y-y));
    //     if(point_distance<min_distance)
    //     {
    //         min_distance = point_distance;
    //         min_distance_pos = i;
    //     }
    //     i++;
    // }
    // local_goal_id=min_distance_pos;
    // if(min_distance>off_track_threshold)
    // {
    //     return false;
    // }
    // else
    // {
    //     return true;
    // }    
    int i =0;
    int min_distance_pos=0;
    double min_distance = 99999;
    while(i<last_published_trajectory.trajectory_points.size())
    {
        double point_distance;
        prius_msgs::My_Trajectory_Point point = last_published_trajectory.trajectory_points[i];
        point_distance = sqrt((point.x-x)*(point.x-x)+(point.y-y)*(point.y-y));
        if(point_distance<min_distance)
        {
            min_distance = point_distance;
        }
        i++;
    }
    if(min_distance>off_track_threshold)
    {
        return false;
    }
    else
    {
        return true;
    }    
}
void Planning::LocalPlanning(const nav_msgs::Odometry &current_localization)
{
    // stop for reverse
    // ROS_INFO_STREAM("stop_count is "<< stop_count);
    // ROS_INFO_STREAM("stop_time is "<< stop_time);

    if(stop_count<stop_time*unit_time_length)
    {
        ++stop_count;
        
        return;
    }


    if(global_trajectory.trajectory_info.empty())
    {
        ROS_INFO("global trajectory is empty");
        return;
    }


    Status localization_status=vehicle_state.GetVehicleStateFromLocalization(current_localization);
    if(localization_status.status != "OK")
    {
        ROS_INFO("Get vehicle state failed!");
        return;
    }

    if(!last_published_trajectory.trajectory_points.empty())
    {
        double x = current_localization.pose.pose.position.x;
        double y = current_localization.pose.pose.position.y;
        if(!IsOnTrajectory(x,y)&&(!vehicle_state.is_off_track))
        {
            ROS_INFO("OFF track");
            vehicle_state.is_off_track = true;
            vehicle_state.is_waiting = false;
        }
        else
        {
            int target_pos;
            if(vehicle_state.is_going_back||vehicle_state.is_avoiding_obstacle||vehicle_state.is_off_track)
            {
                target_pos = last_published_trajectory.trajectory_points.size()-1;
            }
            else
            {
                target_pos = last_published_trajectory.trajectory_points.size()/3;
            }
            //target_pos = last_published_trajectory.trajectory_points.size()-1;
            prius_msgs::My_Trajectory_Point waiting_target = last_published_trajectory.trajectory_points[target_pos];

            double dx = waiting_target.x - x;
            double dy = waiting_target.y -y;
            if(sqrt(dx*dx+dy*dy)<deviation_threshold)
            {
                ROS_INFO("reach target point");
                vehicle_state.is_waiting = false;
                vehicle_state.reach_target_position = true;
                if(vehicle_state.is_going_back)
                {
                    vehicle_state.is_going_back = false;
                }
                if(vehicle_state.is_off_track)
                {
                    vehicle_state.is_off_track=false;
                }
                if(vehicle_state.is_avoiding_obstacle)
                {
                    vehicle_state.is_going_back=true;
                    vehicle_state.is_avoiding_obstacle=false;
                }
                
            }
            
        }



        
    }

    if(vehicle_state.is_waiting)
    {
        //ROS_INFO("waiting until reaching destination");
        return;
    }
    

    //ROS_INFO("get localization");
    // ROS_INFO_STREAM("current x ="<<vehicle_state.movement_state.pose.position.x );
    // ROS_INFO_STREAM("current y ="<<vehicle_state.movement_state.pose.position.y );


    global_trajectory.MatchPointByPosition(vehicle_state);
    prius_msgs::My_Trajectory_Point goal_state = global_trajectory.goal_state;
    //ROS_INFO("match point");


    if(goal_state.v<stop_threshold)
    {
        stop_time = CheckStopTime();
        ROS_INFO_STREAM("STOP_TIME IS "<<stop_time);
        return;
    }
    if(stop_count == stop_time*unit_time_length && stop_time>0)
    {
        stop_count = 0;
        stop_time = 0;
    }




    init_time = (ros::Time::now()).toSec();
    Frame frame;
    frame.Init(planning_conf);
    frame.Update(frame_id,goal_state.x,goal_state.y,init_time,vehicle_state);
    frame.GetReferenceLine(global_trajectory);
    //ROS_INFO_STREAM("reference_line_frenet size is "<<frame.reference_line_frenet.trajectory_points.size());
    //ROS_INFO_STREAM("reference_line size is "<<frame.reference_line.trajectory_points.size());

    //ROS_INFO("start to generate local trajectory");
    prius_msgs::My_Trajectory local_trajectory  = frame.reference_line_frenet;
    float x=vehicle_state.movement_state.pose.position.x;
    float y=vehicle_state.movement_state.pose.position.y;
    // 1. if the distance between vehicle and obstacle is larger than safe-distance + (diagonal of vehicle and obstacle)/2, we have to replan
    for(int i=0;i<obstacle_list.size();++i)
    {
        map<int,Obstacle>::iterator pos = obstacle_list.find(i+1);
        if(!frame.CheckCollision((*pos).second))
        {
            ROS_INFO_STREAM("the "<<(*pos).first<<" obstacle is detected.");
            frame.AddObstacle((*pos).first);
        }
    }
    ROS_INFO_STREAM("num of effective obstacle is "<<frame.effective_obstacles.size());

    if(frame.HaveObstacle()||vehicle_state.is_off_track||vehicle_state.is_going_back)
    {
        if(frame.HaveObstacle())
        {
            ROS_INFO("replan due to obstacles");
        }
        else if(vehicle_state.is_going_back)
        {
            ROS_INFO("replan to go back");
        }
        else
        {
            ROS_INFO("replan dur to off track");            
        }
        

        Replan(local_trajectory,frame);
        //is_replanned = true;
    }
    last_published_trajectory = local_trajectory;
    //ROS_INFO_STREAM("trajectory size is "<<local_trajectory.trajectory_points.size());
    ontime_trajectory_pub.publish(local_trajectory);
    ROS_INFO("publish ontime trajectory");
    // ROS_INFO_STREAM("the size of ontime trajectory is "<<local_trajectory.trajectory_points.size());
    // ROS_INFO_STREAM("local trajectory first x is "<<local_trajectory.trajectory_points[0].x);
    // ROS_INFO_STREAM("local trajectory first y is "<<local_trajectory.trajectory_points[0].y);
    // ROS_INFO_STREAM("local trajectory last x is "<<local_trajectory.trajectory_points.back().x);
    // ROS_INFO_STREAM("local trajectory last y is "<<local_trajectory.trajectory_points.back().y);
    obstacle_pub.publish(obstacle_info);
    //ROS_INFO("publish obstacle successfully!");
    ROS_INFO("**********************************");
}
void Planning::Replan(prius_msgs::My_Trajectory &local_trajectory,Frame &frame)
{
    ROS_INFO("start to replan");
    if(planner_agent.activate_planning_function)
    {
        frame.GetObstacleRange(obstacle_list);
        planner_agent.Plan(vehicle_state, frame,local_trajectory);
        vehicle_state.reach_target_position = false;
        vehicle_state.is_waiting = true;
        if(frame.HaveObstacle())
        {
            vehicle_state.is_avoiding_obstacle = true;
        }
        frame.effective_obstacles.clear();
    }
}
int Planning::CheckStopTime()
{
    int i = global_trajectory.goal_id;
    while(true)
    {
        if(global_trajectory.trajectory_info[i].v<stop_threshold)
        {
            i++;
        }
        else
        {
            break;
        }       
    }
    int stop_time = i-global_trajectory.goal_id;
    global_trajectory.goal_id = i+30;
    return stop_time;
}

void Planning::CreateObstacles(int num)
{
    srand((int)time(0));
    int length = global_trajectory.trajectory_info.size();
    for(int i=0;i<num;++i)
    {
        prius_msgs::ObstacleAttr obstacle_attribute;
        // int id = (random(length/600-2)+2)*600;
        int id = 950*(2*i+1);
        obstacle_attribute.id = id;
        // obstacle_attribute.x = global_trajectory.trajectory_info[id].x+(random(3)-1)*15;
        // obstacle_attribute.y = global_trajectory.trajectory_info[id].y+(random(3)-1)*15;
        obstacle_attribute.x = global_trajectory.trajectory_info[id].x;
        obstacle_attribute.y = global_trajectory.trajectory_info[id].y;
        obstacle_attribute.length = random(5)+3;
        obstacle_attribute.width = random(5)+3;
        obstacle_attribute.heading = (random(180)-90)*PI/180;
        obstacle_attribute.speed = 0.0;
        obstacle_info.push_back(obstacle_attribute);
        Obstacle Obstacle(obstacle_attribute);
        obstacle_list.insert(make_pair(i+1,Obstacle));
        // ROS_INFO_STREAM("the length of the "<<i<<" obstacle is "<<obstacle_attribute.length);
        // ROS_INFO_STREAM("the width of the "<<i<<" obstacle is "<<obstacle_attribute.width);
        // ROS_INFO_STREAM("the heading of the "<<i<<" obstacle is "<<obstacle_attribute.heading);
    }
}


void Planning::InterceptTrajectory(prius_msgs::My_Trajectory &local_trajectory, GlobalTrajectory &global_trajectory)
{
    vector<prius_msgs::My_Trajectory_Point>::iterator start =global_trajectory.trajectory_info.begin()+global_trajectory.goal_id;
    for(vector<prius_msgs::My_Trajectory_Point>::iterator it=start;it!=start+local_trajectory_length&&it!=global_trajectory.trajectory_info.end();it++)
    {
        local_trajectory.trajectory_points.push_back(*it);
    }
}