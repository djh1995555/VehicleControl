#pragma once
#include <iostream>
using namespace std;


class PIDController
{
    public:
    void Init(double kp, double ki, double kd);
    double m_kp;
    double m_ki;
    double m_kd;
    double previous_error_=0;
    double previous_output_ = 0.0;
    double preprevious_error_=0.0;
    
    double preprevious_output_ = 0.0;

    bool differentiator_enabled_=true;
    bool integrator_enabled_ = true;
    double integral_ = 0.0; // 积分初值
    bool integrator_hold_ = true; // 设置积分饱和
    double integrator_saturation_high_=0.05;
    double integrator_saturation_low_=-0.05;
    int integrator_saturation_status_ = 0;

    
    int control_position(double error, double &compensation, double dt);
    int control_increment(double error, double &compensation, double dt);
    void AdjustKp(int multiplier);
    void AdjustKi(int multiplier);
    void AdjustKd(int multiplier);

};