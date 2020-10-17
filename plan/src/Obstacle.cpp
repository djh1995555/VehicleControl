#include<planning/Obstacle.h>

double Obstacle::ComputeDistance(double a,double b)
{
    double dx = x-a;
    double dy = y-b;
    return sqrt(dx*dx+dy*dy);
}

double Obstacle::GetX()
{
    return x;
}
double Obstacle::GetY()
{
    return y;
}
double Obstacle::GetHeading()
{
    return ObstacleBox.heading;
}
double Obstacle::GetSpeed()
{
    return speed;
}