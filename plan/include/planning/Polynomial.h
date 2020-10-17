#pragma once
#include <iostream>
using namespace std;
#include <ros/ros.h>
#include <planning/FrenetPath.h>
#include <cmath>



class Polynomial
{
    public:
    void Init(vector<double> coef)
    {
        coefficient = coef;
    }
    double CalculateValue(double param);
    double CalculateFirstDerivative(double param);
    double CalculatesecondDerivative(double param);
    double CalculatethirdDerivative(double param);

    vector<double> coefficient;
};

struct polynomials
{
    Polynomial longitudinal_poly;
    Polynomial lateral_poly;
    FrenetPath frenet_path;
};