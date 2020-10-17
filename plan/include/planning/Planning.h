#pragma once
#include <iostream>
using namespace std;
#include <ros/ros.h>
#include <planning/PlanningConf.h>
#include <planning/GlobalTrajectory.h>
#include <planning/Planner/PlannerAgent.h>
#include <cstdlib>
#include <ctime>
#include <planning/Obstacle.h>
#include <prius_msgs/Obstacles.h>
#include <prius_msgs/ObstacleAttr.h>
#include <vector>
#include <map>
#include <planning/Frame.h>

#define random(x) rand()%(x)
const string PLAN_CONF_FILE_NAME = "/home/djh/ControlModule/src/plan/conf/planning_conf.txt";

class Planning
{
    public:
    Planning()
    {
        ROS_INFO("Planning Node had been created!");
    }
    Status Init();
    Status Start();
    void Stop();
    int Spin();
    void TrajectoryCallback(const prius_msgs::My_Trajectory &routing);
    void LocalPlanning(const nav_msgs::Odometry &current_localization);
    void CreateObstacles(int num);
    int CheckStopTime();
    void InterceptTrajectory(prius_msgs::My_Trajectory &local_trajectory, GlobalTrajectory &global_trajectory);
    void Replan(prius_msgs::My_Trajectory &local_trajectory,Frame &frame);
    bool IsOnTrajectory(const double x,const double y);

    int local_goal_id=0;
    int local_search_length = 100;

    int frame_id =0;
    int count=0;
    int stop_count=0;
    int stop_time=0;
    
    const int deviation_threshold=10;
    const int off_track_threshold=5;
    const double safe_distance=30;
    const int search_length=500;
    const int local_trajectory_length = 200;
    const double stop_threshold =0.05;
    const int unit_time_length=1;
    double init_time;
    map<int, Obstacle> obstacle_list;
    vector<prius_msgs::ObstacleAttr> obstacle_info;
    prius_msgs::My_Trajectory last_published_trajectory;
    

    private:
    ros::NodeHandle node;
    ros::Publisher ontime_trajectory_pub;
    ros::Publisher obstacle_pub;
    ros::Subscriber global_trajectory_sub;
    ros::Subscriber localization_sub;
    Status status;
    VehicleState vehicle_state;
    PlanningConf planning_conf;
    PlannerAgent planner_agent;
    GlobalTrajectory global_trajectory;
    //Frame frame;

};