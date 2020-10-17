#pragma once
#include <iostream>
#include <prius_msgs/ObstacleAttr.h>
#include <planning/BoundingBox.h>
#include <math.h>
#include <planning/FrenetPath.h>
using namespace std;





class Obstacle
{
    public:
    Obstacle(const prius_msgs::ObstacleAttr attr):ObstacleBox(attr.length,attr.width,1)
    {
        ObstacleBox.SetHeading(attr.heading);
        ObstacleBox.GetDiagonal();
        id = attr.id;
        x = attr.x;
        y = attr.y;
        speed = attr.speed;
    }
    
    double ComputeDistance(double x,double y);
    double GetX();
    double GetY();
    double GetHeading();
    double GetSpeed();

    BoundingBox ObstacleBox;
    FrenetPathPoint frenet_pos;
    bool is_avoided=false;

    private:
    int id;
    double x;
    double y;
    double speed;

};