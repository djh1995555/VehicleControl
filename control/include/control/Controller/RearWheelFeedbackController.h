#pragma once
#include <iostream>
using namespace std;
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <prius_msgs/My_Trajectory.h>

class RearWheelFeedbackController:public Controller
{
    public:
    Status Init(const ControlConf &control_conf);
    Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, const VehicleState &vehicle_state,prius_msgs::Control &control_cmd);

    double Ts;
    prius_msgs::My_Trajectory_Point goal_state;
    double yaw_error_gain;
    double lateral_error_gain;
    double preview_length=20;
    double wheel_base;
    double current_velocity;
    double heading_error;
    double lateral_error;
/*     double heading_error_threshold=0.000000001;
    double lateral_error_threshold=0.1; */

    
    double steering_angle;
    double pi =3.1415926;

};