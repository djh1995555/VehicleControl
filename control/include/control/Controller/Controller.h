#pragma once
#include <iostream>
using namespace std;
#include <control/ControlConf.h>
#include <control/TrajectoryAnalyzer.h>
#include <control/VehicleState.h>
#include <prius_msgs/Control.h>




class Controller
{
    public:
    virtual Status Init(const ControlConf &control_conf)=0;
    virtual Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const VehicleState &vehicle_state,prius_msgs::Control &control_cmd)=0;
    string GetName();
    Status Reset();
    void Stop();

    string name;
    Status status;
};
