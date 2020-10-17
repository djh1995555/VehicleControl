#pragma once
#include <iostream>
#include <math.h>
using namespace std;


class BoundingBox
{
    public:
    BoundingBox(){}
    BoundingBox(double L,double W,double H)
    {
        length = L;
        width = W;
        height = H;
    }
    double GetDiagonal();
    void SetHeading(double yaw);
    double diagonal;

    double heading;
    double length;
    double width; 
    double height;

};