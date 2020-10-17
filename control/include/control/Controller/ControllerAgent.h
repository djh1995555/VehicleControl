#pragma once
#include <iostream>
using namespace std;

#include <control/TrajectoryAnalyzer.h>
#include <control/VehicleState.h>
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <control/Controller/DoublePID.h>
#include<control/Controller/FeedforwardPID.h>
#include <control/Controller/PurePursuit.h>
#include <control/Controller/MPCController.h>
#include <control/Controller/LQRController.h>
#include<control/Controller/StanleyController.h>
#include<control/Controller/RearWheelFeedbackController.h>

#include <algorithm>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <math.h>
#include <prius_msgs/Control.h>
#include <prius_msgs/My_Trajectory.h>
#include <prius_msgs/Augmented_My_Trajectory_Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <fstream>




class ControllerAgent
{
    public:
    ControllerAgent()
    {
        ROS_INFO("Controller Agent is created!");
    }
    Status Init(const ControlConf &control_conf);
    Status ComputeControlCmd( TrajectoryAnalyzer &trajectory_analyzer,const VehicleState &vehicle_state,prius_msgs::Control &control_cmd);
    Status RegisterController(const ControlConf &control_conf);
    void CurrentController();
    void ClampLongitudinalCmd(const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd);
    void ClampLateralCmd(const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd);


    vector<Controller*> controller_list;
    int controller_num;
    string longitudinal_controller;
    string lateral_controller;
    string centralized_controller;

    private:
    Status status;
    double Ts;
};