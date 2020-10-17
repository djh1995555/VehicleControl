#pragma once
#include <iostream>
using namespace std;
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>
using namespace Eigen;

class LQRController:public Controller
{
    public:
    Status Init(const ControlConf &control_conf);
    Status ComputeControlCmd( TrajectoryAnalyzer &trajectory_analyzer, const VehicleState &vehicle_state,prius_msgs::Control &control_cmd);
    void ComputeStateError(const prius_msgs::My_Trajectory_Point &goal_state,const  VehicleState &vehicle_state);
    void SolveLQRProblem(const double tolerance,const int max_iteration_num);
    double GetPreviewSteeringAngle(const prius_msgs::My_Trajectory_Point point);
    void LQRSolverWithPrediction(TrajectoryAnalyzer &trajectory_analyzer,const VehicleState &vehicle_state);
    void LQRSOlver(const prius_msgs::My_Trajectory_Point &goal_state);
    void UpdateKineticModel(double current_velocity,double heading,double steering_angle);

    double Ts;
    double wheel_base;
    double max_steer;
    double LQR_preview_length;
    double LQR_prediction_iteration_num;
    double preview_feedforward_coefficient;
    double solve_tolerance;
    double max_iteration_num;
    double Q_0_0;
    double Q_1_1;
    double Q_2_2;
    double prediction_enabled;
    double previous_steering_angle=0.0;
    

    int Nx=3;
    int Nu=1;

    vector<prius_msgs::My_Trajectory_Point> LQR_preview_trajectory;

    MatrixXd A=MatrixXd::Identity(Nx,Nx);
    MatrixXd B=MatrixXd::Zero(Nx,Nu);
    MatrixXd Q=MatrixXd::Identity(Nx,Nx);
    MatrixXd R=MatrixXd::Zero(Nu,Nu);
    
    MatrixXd K=MatrixXd::Zero(Nu,Nx);
    MatrixXd Kv=MatrixXd::Zero(Nu,Nx);
    MatrixXd Ku=MatrixXd::Zero(Nu,Nu);

    MatrixXd Pk=MatrixXd::Identity(Nx,Nx);
    MatrixXd Vk=MatrixXd::Zero(Nx,Nu);
    MatrixXd Pk_new=Matrix3d::Identity(Nx,Nx);
    MatrixXd Vk_new=MatrixXd::Zero(Nx,Nu);

    MatrixXd state_error = MatrixXd::Zero(Nx,Nu);

    MatrixXd feedforward=MatrixXd::Zero(Nu,1);
    MatrixXd feedback=MatrixXd::Zero(Nu,1);


    double current_velocity=0.0;
    double current_heading=0.0;
    double current_steering_angle=0.0;
    double steering_angle=0.0;

    double pi = 3.1415926;



};
