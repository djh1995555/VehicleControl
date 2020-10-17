#pragma once
#include <iostream>
using namespace std;
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <control/Controller/PIDController.h>

class DoublePID:public Controller
{
    public:
    Status Init(const ControlConf &control_conf);
    Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd);


    PIDController velocity_PID;
    PIDController acceleration_PID;
 

    double acc_compensation;
    double velocity_compensation;
    double slope_compensation;
    double acc_error;
    double velocity_error;

    double torque_cmd;

    double Ts;
    double velocity_kp;
    double velocity_ki;
    double velocity_kd;
    double acc_kp;
    double acc_ki;
    double acc_kd;

};


