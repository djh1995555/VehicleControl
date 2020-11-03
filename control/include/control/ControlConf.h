#pragma once
#include <ros/ros.h>
#include <iostream>
using namespace std;
#include <fstream>
#include<map>

class Status
{
    public:
    string status;
};

const int LOCALIZATION_FREQUENCY=100;

struct ConfParam
{
    // ControlConf
    double Ts;
    // controller_agent
    double controller_num;
    // Double PID
    double velocity_kp;
    double velocity_ki;
    double velocity_kd;
    double acc_kp;
    double acc_ki;
    double acc_kd;
    // Feedforward PID
    double feedforward_enable;
    double slope_compensation_enable;
    double feedforward_gain;
    double detection_length;
    double error_threshold;
    double ff_kp;
    double ff_ki;
    double ff_kd;
    // purepursuit
    double pure_pursuit_preview_length; 
    // Stanley
    double velocity_gain;
    // Rear Wheel Feedback
    double yaw_error_gain;
    double lateral_error_gain;
    // LQR
    double LQR_preview_length;
    double LQR_prediction_iteration_num;
    double preview_feedforward_coefficient;
    double prediction_enabled;
    double LQR_R;
    double LQR_Q_0_0;
    double LQR_Q_1_1;
    double LQR_Q_2_2;
    double solve_tolerance;
    double max_iteration_num;
     // MPC
    double preview_length;
    double MPC_R_0_0;
    double MPC_R_1_1;
    double MPC_Q_0_0;
    double MPC_Q_1_1;
    double MPC_Q_2_2;
    double MPC_control_length;
    double MPC_predict_length;   
    
    // trajectory_analyzer
    double search_length;



    // Vehicle state estimation
    double slope_threshold;
    double pitch_deque_length;
    double filter_method;
    // Physical Attributes
    double G;
    double mass;
    double friction_coefficient;
    double wheel_base;
    double front_wheel_base;
    double rear_wheel_base;
    double front_track;
    double rear_track;
    double wheel_radius;
    double Ix;
    double Iy;
    double Iz;
    double brake_distance_coefficient;
    double chassis_aero_force_gain;
    double max_front_torque;
    double max_back_torque;
    double max_front_brake_torque;
    double max_back_brake_torque;
    double max_speed;
    double max_steer;
    double gas_efficiency;
    double battery_charge_watt_hours;
    double battery_discharge_watt_hours;
    double flwheel_steering_p_gain;
    double frwheel_steering_p_gain;
    double flwheel_steering_i_gain;
    double frwheel_steering_i_gain;
    double flwheel_steering_d_gain;
    double frwheel_steering_d_gain;
    
};

class ControlConf
{
    public:
    ControlConf()
    {
        ROS_INFO("Control_conf is created!");

    }
    Status ReadControlConf(const string control_conf_file_name);
    void printData();   

    ifstream ifs;
    ConfParam conf_param;

    string LONGITUDINAL_CONTROLLER;
    string LATERAL_CONTROLLER;
    string CENTRALIZED_CONTROLLER;


    private:
    Status status;
};

