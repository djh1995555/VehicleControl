#pragma once
#include <iostream>
using namespace std;
#include <ros/ros.h>
#include <planning/FrenetPath.h>
#include <planning/Polynomial.h>

class TrajectoryGenerator
{
    public:
    void GenerateTrajectory(const FrenetPathPoint &start_point,const FrenetPathPoint &end_point,polynomials &polys);
    void GetPolynomial(double start_x,double start_x_d,double start_x_dd,double end_x,double end_x_d,double end_x_dd,double param,Polynomial &poly);
    void CombineTrajectory(const FrenetPathPoint &start_point,const FrenetPathPoint &end_point,polynomials &polys);
    double ComputeCost(FrenetPath &frenet_path,const double target_speed);


    double time_interval = 0.01;
    double K_J = 0.1;
    double K_T = 0.1;
    double K_D =1;
    double K_lon = 1;
    double K_lat = 1;
};