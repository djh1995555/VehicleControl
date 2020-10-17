#pragma once
#include <iostream>
using namespace std;
#include <ros/ros.h>
#include <planning/PlanningConf.h>
#include <planning/Planner/Planner.h>
#include <planning/FrenetPath.h>
#include <planning/TrajectoryGenerator.h>



class LatticePlanner:public Planner
{
    public:
    Status Init(const PlanningConf &planning_conf);
    Status PlanOnReferenceLine(VehicleState &vehicle_state, Frame &frame,prius_msgs::My_Trajectory &local_trajectory);
    void CreateEndState(VehicleState &vehicle_state,Frame &frame);
    void GenerateTrajectory(const FrenetPathPoint &start_point,const FrenetPathPoint &end_point,vector<polynomials> &polynomial_set);
    void CollisionCheck(polynomials &polys,Frame &frame,VehicleState &vehicle_state);
    void GetBestTrajectory(Frame &frame);

    TrajectoryGenerator trajectory_generator;
    double target_velocity;
    double lateral_sample_num;
    double lateral_sample_interval;
    double station_sample_num;
    double station_sample_interval;
    double time_sample_num;
    double time_sample_interval;
};

