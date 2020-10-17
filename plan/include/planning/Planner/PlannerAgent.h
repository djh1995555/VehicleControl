#pragma once
#include <iostream>
using namespace std;
#include <ros/ros.h>
#include <planning/PlanningConf.h>
#include <planning/Planner/Planner.h>
#include <planning/Planner/LatticePlanner.h>
#include <prius_msgs/My_Trajectory.h>
#include <planning/VehicleState.h>
#include <planning/Frame.h>



class PlannerAgent
{
    public:
    PlannerAgent()
    {
        ROS_INFO("Planner Agent is created!");
    }
    Status Init(const PlanningConf &planning_conf);
    Status Plan(VehicleState &vehicle_state, Frame &frame,prius_msgs::My_Trajectory &local_trajectory);
    Status RegisterPlanner(const PlanningConf &planning_conf);
    void CurrentPlanner();

    vector<Planner*> planner_list;
    string planner_name;
    double activate_planning_function;

    private:
    Status status;
    double Ts;


   
};