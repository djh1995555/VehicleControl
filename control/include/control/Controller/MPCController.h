#pragma once
#include <iostream>
using namespace std;
#include "qpOASES.hpp"
#include <control/ControlConf.h>
#include <control/Controller/Controller.h>
#include <prius_msgs/Control.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>

using namespace qpOASES;
using namespace Eigen;

class MPCController:public Controller
{
    public:
    Status Init(const ControlConf &control_conf);
    Status ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, const VehicleState &vehicle_state,prius_msgs::Control &control_cmd);
    void GetAugmentedMatrix(MatrixXd &matrix_augmented,const MatrixXd &matrix,int num);
    void ComputeStateError(const prius_msgs::My_Trajectory_Point &goal_state,const VehicleState &vehicle_state);
    void CreatePredictionModel();
    double GetPlannedSteeringAngle(const prius_msgs::My_Trajectory_Point point);


    double pi=3.1415926;

    int Nx=3; // 状态量个数
    int Nu=1; // 控制量个数
    double current_velocity=0.0;
    double planned_heading=0.0;
    double planned_steering_angle=0.0;
    double planned_velocity=0.0;

    double prediction_length;
    double control_length;
    double wheel_base;
    double previous_steering_angle=0.0;
/*     vector<double> V_Q;
    vector<double> V_R; */
    double Ts;

    MatrixXd A=MatrixXd::Identity(Nx,Nx);
    MatrixXd B=MatrixXd::Zero(Nx,Nu);
    MatrixXd Q=MatrixXd::Identity(Nx,Nx);
    MatrixXd R=MatrixXd::Identity(Nu,Nu);
    MatrixXd Q_augmented;
    MatrixXd R_augmented;

    MatrixXd A_augmented;
    MatrixXd B_augmented;


    VectorXd state_error = VectorXd::Zero(Nx);
    VectorXd control_output=VectorXd::Zero(Nu);


};
