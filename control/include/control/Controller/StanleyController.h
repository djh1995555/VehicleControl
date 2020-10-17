#pragma once
#include <iostream>
using namespace std;
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <prius_msgs/My_Trajectory.h>

class StanleyController:public Controller
{
    public:
    Status Init(const ControlConf &control_conf);
    Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd);
    void TransformToFrontAxle(double x,double y);



    double Ts;
    double velocity_gain;
    prius_msgs::My_Trajectory_Point goal_state;
    
    double front_axle_x;
    double front_axle_y;
    double current_heading;
    double current_velocity;
    double steering_angle;
    double wheel_base;
    double pi=3.141596;

};