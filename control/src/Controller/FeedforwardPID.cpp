#include<control/Controller/FeedforwardPID.h>

Status FeedforwardPID::Init(const ControlConf &control_conf)
{
    name = "Feedforwar PID controller";
    Ts=control_conf.conf_param.Ts;
    original_feedforward_gain=control_conf.conf_param.feedforward_gain;
    error_threshold=control_conf.conf_param.error_threshold;
    kp=control_conf.conf_param.ff_kp;
    ki=control_conf.conf_param.ff_ki;
    kd=control_conf.conf_param.ff_kd;
    feedforward_enable=control_conf.conf_param.feedforward_enable;
    slope_compensation_enable=control_conf.conf_param.slope_compensation_enable;
    detection_length=control_conf.conf_param.detection_length;
    feedforward_gain = 0.0;
    for(int i =0;i<detection_length;++i)
    {
        errors.push_back(0);
    }
    velocity_PID.Init(kp,ki,kd);
    ROS_INFO_STREAM(name<<" is initialized successfully!");
    status.status = "OK";
    return status;

}
double FeedforwardPID::GetMean(deque<double> &errors)
{
    double sum;
    for(int i=0;i<detection_length;++i)
    {
        sum +=errors[i];
    }
    return sum/detection_length;
}
Status FeedforwardPID::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    
    ROS_INFO("Use Feedforward PID");
    velocity_error = vehicle_state.velocity_error;
    if(feedforward_enable==1)
    {
        errors.pop_front();
        errors.push_back(velocity_error);
        double average_error=GetMean(errors);
        if(abs(average_error)<error_threshold)
        {
            feedforward_gain=0;
            gain_changed=false;
        }
        else if(abs(average_error)>error_threshold&&gain_changed==false)
        {
            if(average_error>0)
            {
                feedforward_gain=original_feedforward_gain;
                //velocity_PID.AdjustKp(2);
                gain_changed=true;
            }
            else
            {
                feedforward_gain=-0.2*original_feedforward_gain;
                //velocity_PID.AdjustKp(1);
                gain_changed=true;           
            }
        }
    }
    if(slope_compensation_enable==1)
    {
        double slope = vehicle_state.slope;
        //ROS_INFO_STREAM("slope = "<<slope);
        slope_compensation = vehicle_state.mass*vehicle_state.G*sin(slope*PI/180)/(vehicle_state.wheel_radius*23);
        //ROS_INFO_STREAM("slope_compensation = "<<slope_compensation);
    }
    velocity_PID.control_increment(velocity_error,feedback_compensation,Ts);
    torque_cmd = feedforward_gain*trajectory_analyzer.goal_state.v+feedback_compensation+slope_compensation;
    //ROS_INFO_STREAM("torque_cmd="<<torque_cmd);


    control_cmd.throttle=torque_cmd;

    //ROS_INFO("Compute longitudinal command successfully!");

    status.status = "OK";
    return status;
}