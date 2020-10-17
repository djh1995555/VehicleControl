#pragma once
#include <iostream>
using namespace std;
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <prius_msgs/My_Trajectory.h>

class PurePursuit:public Controller
{
    public:
    Status Init(const ControlConf &control_conf);
    Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd);


    double alpha;
    double pure_pursuit_preview_length;
    double Ts;
    prius_msgs::My_Trajectory_Point preview_state;
    double preview_distance;
    
    double steering_angle;

};