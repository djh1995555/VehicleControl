#pragma once
#include <iostream>
using namespace std;

#include <control/Controller/PIDController.h>
#include <control/Controller/ControllerAgent.h>
#include <control/TrajectoryAnalyzer.h>
#include <control/VehicleState.h>
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <control/Controller/DoublePID.h>
#include <control/Controller/LQRController.h>
#include <control/Controller/MPCController.h>
#include <control/Controller/PurePursuit.h>
#include <control/SignalDefines.hpp>

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
#include <prius_msgs/VehicleInfo.h>

const string CONTROL_CONF_FILE_NAME = "/home/djh/ControlModule/src/control/conf/control_conf.txt";

class Control
{
    public:
    Control()
    {
        ROS_INFO("Control Node had been created!");
    }
    Status Init();
    Status Start();
    void Stop();
    int Spin();
    void TrajectoryCallback(const prius_msgs::My_Trajectory &trajectory);
    void LocalizationCallback(const nav_msgs::Odometry &localization);
    void ChassisInfoCallback(const prius_msgs::VehicleInfo &chassis_info);
    void ChassisStateCallback(const nox_msgs::SignalArray::ConstPtr &chassis_state);
    void CARSIMCallback(const nav_msgs::Odometry &carsim_feedback);
    void WriteInDebug(ofstream &ofs, prius_msgs::Augmented_My_Trajectory_Point vehicle_info,TrajectoryAnalyzer &trajectory_analyzer,ControllerAgent &controller_agent);

    
    private:
    int count =0;
    int frequency =10;
    string simulation_platform;
    double init_time = 0.0;
    int timestamp_num = 0;
    int timestamp_range = 5;
    double start_time=0.0;
    double end_time=0.0;
    double last_computational_time=0.0;
    double time_difference_threshold=10;
    double computational_time=0.0;
    double average_computational_time=0.0;
    double max_computational_time=0.0;
    int control_cycle_num=0;
    Status status;
    ControlConf control_conf;
    ControllerAgent controller_agent;
    TrajectoryAnalyzer trajectory_analyzer;
    VehicleState vehicle_state;
    prius_msgs::Control control_cmd;
    int received_trajectory_num=0;
    
    ofstream ofs;
    string simulation_result="/home/djh/ControlModule/src/control/plot_py/simulation_result.txt";


    ros::NodeHandle node;
    ros::Publisher command_pub;
    ros::Publisher carsim_pub;
    ros::Publisher vehicle_info_pub;
    ros::Subscriber trajectory_sub;
    ros::Subscriber localization_sub;
    ros::Subscriber chassis_state_sub;
    ros::Subscriber carsim_sub;
    ros::Subscriber chassis_info_sub;
};