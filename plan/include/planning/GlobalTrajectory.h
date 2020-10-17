#pragma once
#include <iostream>
using namespace std;
#include <ros/ros.h>
#include <planning/PlanningConf.h>
#include <prius_msgs/My_Trajectory.h>
#include <planning/VehicleState.h>


class GlobalTrajectory
{
    public:
    GlobalTrajectory()
    {
        ROS_INFO("GlobalTrajectory  is created!");
    }
    Status Init(const PlanningConf &planning_conf);
    Status ReadTrajectory(const prius_msgs::My_Trajectory &trajectory);
    void PrintTrajectory();
    Status MatchPointByPosition(VehicleState &vehicle_state);
    double ComputeDist(const prius_msgs::My_Trajectory_Point trajectory_point, double x, double y);
    double ComputeDist(const prius_msgs::My_Trajectory_Point trajectory_point_a, const prius_msgs::My_Trajectory_Point trajectory_point_b);

    int goal_id = 0;
    int search_length;
    double Ts;
    vector<prius_msgs::My_Trajectory_Point> trajectory_info;
    prius_msgs::My_Trajectory_Point destination;
    prius_msgs::My_Trajectory_Point goal_state;

    private:
    Status status;
};