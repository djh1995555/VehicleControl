#pragma once
#include <iostream>
using namespace std;
#include <ros/ros.h>
#include <planning/PlanningConf.h>
#include <planning/VehicleState.h>
#include <planning/Frame.h>

class Planner
{
    public:
    virtual Status Init(const PlanningConf &planning_conf)=0;
    virtual Status PlanOnReferenceLine(VehicleState &vehicle_state, Frame &frame,prius_msgs::My_Trajectory &local_trajectory)=0;
    string GetName();
    Status Reset();
    void Stop();

    string name;
    Status status;
};