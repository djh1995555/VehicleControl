#pragma once
#include <ros/ros.h>
#include <iostream>
using namespace std;
#include <fstream>
#include <map>

class Status
{
    public:
    string status;
};

const int LOCALIZATION_FREQUENCY=100;


struct ConfParamPrivate
{
    double Ts;
    double target_velocity;
    double lateral_sample_num;
    double lateral_sample_interval;
    double station_sample_num;
    double station_sample_interval;
    double time_sample_num;
    double time_sample_interval;
    double Activate_planning_function;
    double search_length;
    double obstacles_num;
    double frame_forward_time;
    double frame_safe_distance;
    double frame_forward_distance_limit;
    double speed_deque_length;
    // Physical Attributes
    double G;
    double mass;
    double length;
    double width;
    double height;
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

class PlanningConf
{
    public:
    PlanningConf()
    {
        ROS_INFO("PlanningConf is created!");

    }
    Status ReadControlConf(const string control_conf_file_name);
    void printData();   

    ifstream ifs;
    ConfParamPrivate planning_param;

    string Planner_name;

    private:
    Status status;
};

