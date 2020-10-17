#include <planning/GlobalTrajectory.h>

Status GlobalTrajectory::Init(const PlanningConf &planning_conf)
{
    Ts=planning_conf.planning_param.Ts;
    search_length=planning_conf.planning_param.search_length;
    ROS_INFO("global trajectory is initialized!");
    status.status = "OK";
    return status;
}

Status GlobalTrajectory::ReadTrajectory(const prius_msgs::My_Trajectory &trajectory)
{
    for(prius_msgs::My_Trajectory_Point point : trajectory.trajectory_points)
    {
        trajectory_info.push_back(point);
    }
    destination = trajectory_info.back();
    status.status = "OK";
    return status;
}

void GlobalTrajectory::PrintTrajectory()
{
    //ROS_INFO("Publish the trajectory!");
    for(int i =trajectory_info.size()-1 ;i>100;i=i-1000)
    {
        prius_msgs::My_Trajectory_Point point = trajectory_info[i];
        ROS_INFO_STREAM("x="<< point.x);
        ROS_INFO_STREAM("y=" << point.y);
        ROS_INFO_STREAM("z=" << point.z);
    }
}

Status GlobalTrajectory::MatchPointByPosition(VehicleState &vehicle_state)
{
    //ROS_INFO("start to find nearest point");
    double x=vehicle_state.movement_state.pose.position.x;
    double y=vehicle_state.movement_state.pose.position.y;
    
    vector<double> dist;
    int i =goal_id;
    while(i<goal_id+search_length&&i<trajectory_info.size())
    {
        double point_distance;
        point_distance = ComputeDist(trajectory_info[i],x,y);
        dist.push_back(point_distance);
        i++;
    }
    vector<double>::iterator goal=min_element(dist.begin(), dist.end());
    vehicle_state.distance_error = *goal;
    goal_id=distance(dist.begin(), goal)+goal_id;
    //ROS_INFO_STREAM("goal_id is "<<goal_id);
    goal_state = trajectory_info[goal_id];

    status.status = "OK";
    return status;   
}

double GlobalTrajectory::ComputeDist(const prius_msgs::My_Trajectory_Point trajectory_point, double x, double y)
{
    float distance;
    return distance = sqrt((trajectory_point.x-x)*(trajectory_point.x-x)+(trajectory_point.y-y)*(trajectory_point.y-y));
}

double GlobalTrajectory::ComputeDist(const prius_msgs::My_Trajectory_Point point_a, const prius_msgs::My_Trajectory_Point point_b)
{
    float distance;
    return distance = sqrt((point_a.x-point_b.x)*(point_a.x-point_b.x)+(point_a.y-point_b.y)*(point_a.y-point_b.y));
}