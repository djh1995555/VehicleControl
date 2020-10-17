#pragma once
#include <iostream>
using namespace std;
#include <ros/ros.h>

struct FrenetPathPoint
{
    double s;
    double s_d;
    double s_dd;
    double s_ddd;
    double d;
    double d_d;
    double d_dd;
    double d_ddd;
    double t;
};

class FrenetPath
{
    public:
    // FrenetPath()
    // {
    //     ROS_INFO("Frenet path is created!");
    // }
    void ComputeCost();

    vector<FrenetPathPoint> FrenectPoints;
    double cost;
    bool is_collided=false;
    bool is_valid = true;
};

