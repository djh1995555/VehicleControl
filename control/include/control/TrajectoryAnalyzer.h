#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
using namespace std;
#include <control/ControlConf.h>
#include <control/VehicleState.h>
#include <prius_msgs/My_Trajectory.h>


class TrajectoryAnalyzer
{
    public:
    TrajectoryAnalyzer()
    {
        ROS_INFO("Trajectory analyzer is created!");
    }
    Status Init(const ControlConf &control_conf);
    Status ReadTrajectory(const prius_msgs::My_Trajectory &trajectory);
    double ComputeDist(const prius_msgs::My_Trajectory_Point path_point, double x, double y);
    Status MatchPointByPosition(VehicleState &vehicle_state);
    double MatchPointByPositionForStanley(double x,double y);
    void GetPreviewPoint(double preview_length);
    vector<prius_msgs::My_Trajectory_Point> GetPreviewTrajectory(double preview_length);
    void PrintTrajectory();


    prius_msgs::My_Trajectory_Point destination;
    vector<prius_msgs::My_Trajectory_Point> trajectory_info;
    prius_msgs::My_Trajectory_Point goal_state;  
    prius_msgs::My_Trajectory_Point preview_state; 
    int goal_id=0;
    int stanley_goal_id = 0;
    int preview_id=0;
    bool ESTOP = false;

    
    int search_length;
    double Ts;


    private:
    Status status;
    
};