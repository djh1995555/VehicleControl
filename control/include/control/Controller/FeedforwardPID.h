#pragma once
#include <iostream>
using namespace std;
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <control/Controller/PIDController.h>
#include <deque>

class FeedforwardPID:public Controller
{
    public:
    Status Init(const ControlConf &control_conf);
    Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd);
    double GetMean(deque<double> &errors);


    PIDController velocity_PID;
    deque<double> errors;
    bool gain_changed=false;
    double detection_length;
    double error_threshold;
    double feedforward_enable;
    double slope_compensation_enable;
    double slope_compensation=0.0;
    double feedback_compensation;
    double feedforward_compensation;
    double torque_cmd;
    double original_feedforward_gain;
    double feedforward_gain;
    double kp;
    double ki;
    double kd;
    double Ts;
    double velocity_error=0.0;


};