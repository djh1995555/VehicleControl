#pragma once
#include <iostream>
using namespace std;
#include <planning/VehicleState.h>
#include <planning/GlobalTrajectory.h>
#include <planning/Obstacle.h>
#include <planning/FrenetPath.h>
#include <planning/Polynomial.h>



class Frame
{
    public:
    Frame()
    {
        ROS_INFO_STREAM("Frame is created!");
    }
    void Init(const PlanningConf &planning_conf);
    void Update(int fid, double x, double y,double time_stamp,VehicleState &v_state);
    void GetReferenceLine(GlobalTrajectory &global_trajectory);
    bool CheckCollision(Obstacle &obstacle);
    void AddObstacle(int obstacle_id);
    bool HaveObstacle();
    void GetObstacleRange(map<int,Obstacle> &obstacle_list);
    int MatchPoint(double x,double y);
    int MatchPointInFrenet(FrenetPathPoint &point);
    void TransformToFrenet(VehicleState &vehicle_state,FrenetPathPoint & frenet_point);
    void TransformToFrenet(Obstacle &obstacle, FrenetPathPoint & frenet_point);
    void TransformToCartesian();
    double NormalizeAngle(double angle);

    prius_msgs::My_Trajectory reference_line_frenet;
    FrenetPathPoint start_point;
    prius_msgs::My_Trajectory_Point origin_point;
    vector<FrenetPathPoint> end_point_set;
    vector<polynomials> polynomial_set;
    polynomials best_trajectory_frenet;
    prius_msgs::My_Trajectory best_trajectory;
    double min_s=9999;
    double min_d=9999;
    double max_s=-9999;
    double max_d=-9999;
    int frenet_goal_id=0;
    double last_x=0;
    double last_y=0;
    double last_yaw=0;

    vector<int> effective_obstacles;

    private:
    double forward_time;
    double forward_distance_limit;
    double safe_distance;
    int frame_id;
    double start_x;
    double start_y;
    double start_time;
    VehicleState vehicle_state;
    Status status;

};