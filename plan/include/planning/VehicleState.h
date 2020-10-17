#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
using namespace std;
#include <planning/PlanningConf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <prius_msgs/My_Trajectory.h>
#include <prius_msgs/Augmented_My_Trajectory_Point.h>
#include <prius_msgs/VehicleInfo.h>
#include <deque>
#include <planning/BoundingBox.h>

const double PI=3.1415926;

struct MovementState
{
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    geometry_msgs::Vector3 acc;
};
class VehicleState
{
    public:
    VehicleState()
    {
        ROS_INFO("Vehicle state is created!");
    }
    Status Init(const PlanningConf &control_conf);
    Status GetVehicleStateFromLocalization(const nav_msgs::Odometry &localization);
    Status GetVehicleStateFromChassis(const prius_msgs::VehicleInfo &current_state);
    void ComputeDistanceFromDestination(const prius_msgs::My_Trajectory_Point destination);
    void ComputeVelocity();
    void ComputeAcc();
    void ComputeEular(double x,double y,double z,double w);
    void ComputeHeadingAngle();
    void ComputeVelocityError(const prius_msgs::My_Trajectory_Point &goal_state);

    


    string state="STOP!";
    MovementState movement_state; 
    prius_msgs::VehicleInfo info_from_chassis;
    prius_msgs::Augmented_My_Trajectory_Point vehicle_info;
    deque<double> pitch_deque;
    BoundingBox VehicleBox;
    deque<double> speed_deque;
    double speed_deque_length;

    bool is_avoiding_obstacle = false;
    bool is_off_track = false;
    bool is_replanned =false;
    bool is_waiting = false;
    bool reach_target_position = false;
    bool is_going_back = false;

    double safe_distance=25;
    double max_deceleration=2;
    double previous_velocity=0.0;
    double current_velocity=0.0;
    double acceleration=0.0;
    double pose_angle[3]={0};
    double heading_angle=0.0;
    double slope=0.0;

    double distance_from_destination=0.0;

    double distance_error=0.0;
    double velocity_error=0.0;
    double acc_error=0.0;

    double heading_error=0.0;
    double lateral_distance_error=0.0;

    double lateral_velocity_error=0.0;
    double yaw_rate_error=0.0;
    double brake_distance_ahead=0.0;

    // vehicle_attributes
    double Ts;
    double slope_threshold;
    double pitch_deque_length;
    double filter_method;
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

    private:
    Status status;
    
};