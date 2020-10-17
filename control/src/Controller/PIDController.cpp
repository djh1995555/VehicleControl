#include <control/Controller/PIDController.h>
#include <ros/ros.h>

void PIDController::Init(double kp, double ki, double kd)
{
    m_kp=kp;
    m_ki=ki;
    m_kd=kd;
    
}
// 位置式PID，有可能会出现积分饱和
int PIDController::control_position(double error, double &compensation,double dt) 
{
    if (dt <= 0) 
    {
        cout<<"dt <= 0, will use the last output, dt: " << dt<<endl;
        return previous_output_;
    }
    double diff = 0;
    double output = 0;


    if(differentiator_enabled_)
    {
        // 也可以只是 error - previous_error_，但是Kd的值不一样
        diff = (error - previous_error_) / dt;
    }
    else
    {
        diff = 0;
    }

    if (integrator_enabled_) 
    {
        // 如果不存在积分项，积分设为0
        if (integrator_hold_) 
        {
        // 也可以是error*ki，但是ki的值不一样，用matlab调参时要注意
            
        // apply Ki before integrating to avoid steps when change Ki at steady state
            if (integral_ + error * m_ki > integrator_saturation_high_) 
            {
                integral_ = integrator_saturation_high_;
                integrator_saturation_status_ = 1;
            } 
            else if (integral_+ error * m_ki < integrator_saturation_low_)
            {
                integral_ = integrator_saturation_low_;
                integrator_saturation_status_ = -1;
            } 
            else 
            {
                integral_ += error * m_ki;
                integrator_saturation_status_ = 0;
            }
        }
    } 

    
    output = error * m_kp + integral_ + diff * m_kd;
    previous_error_ = error;
    previous_output_ = output;
    compensation = output;
    //ROS_INFO("compute compensation(pos) successfully!");
    return 0;
}

// 增量式PID，可以避免积分饱和的情况
int PIDController::control_increment(double error, double &compensation, double dt)
{

    if (dt <= 0) 
    {
        cout<<"dt <= 0, will use the last output, dt: " << dt<<endl;
        return previous_output_;
    }
    double diff = 0;
    double output = 0;

    if(differentiator_enabled_)
    {
        // 也可以只是 error - previous_error_，但是Kd的值不一样
        diff = (error - 2*previous_error_+preprevious_error_) / dt;
    }
    else
    {
        diff = 0;
    }
    

    if (integrator_enabled_) 
    {
        
        integral_=m_ki*error;

    } 
    else
    {
        // 如果不存在积分项，积分设为0
        integral_ = 0;
    }
    
    float increment = (error-previous_error_) * m_kp + integral_ + diff * m_kd;
    preprevious_error_=previous_error_;
    previous_error_ = error;
    output = previous_output_+increment;
    previous_output_ = output;
    compensation = output;
    //ROS_INFO("compute compensation(inc) successfully!");
    return 0;
}

void PIDController::AdjustKp(int multiplier)
{
    m_kp=m_kp*multiplier;
}
void PIDController::AdjustKi(int multiplier)
{
    m_ki=m_ki*multiplier;
}
void PIDController::AdjustKd(int multiplier)
{
    m_kd=m_kd*multiplier;
}