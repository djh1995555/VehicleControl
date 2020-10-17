#include <control/Controller/DoublePID.h>

Status DoublePID::Init(const ControlConf &control_conf)
{
    name = "Double PID controller";
    Ts=control_conf.conf_param.Ts;
    velocity_kp=control_conf.conf_param.velocity_kp;
    velocity_ki=control_conf.conf_param.velocity_ki;
    velocity_kd=control_conf.conf_param.velocity_kd;
    acc_kp=control_conf.conf_param.acc_kp;
    acc_ki=control_conf.conf_param.acc_ki;
    acc_kd=control_conf.conf_param.acc_kd;
    
    
    velocity_PID.Init(velocity_kp,velocity_ki,velocity_kd);
    acceleration_PID.Init(acc_kp,acc_ki,acc_kd);
    ROS_INFO_STREAM(name<<" is initialized successfully!");
    status.status = "OK";
    return status;

}
Status DoublePID::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    //ROS_INFO("Use DoublePID");
    ROS_INFO_STREAM("VP = "<<velocity_PID.m_kp);
    ROS_INFO_STREAM("VI = "<<velocity_PID.m_ki);
    ROS_INFO_STREAM("VD = "<<velocity_PID.m_kd);
    ROS_INFO_STREAM("AP = "<<acceleration_PID.m_kp);
    ROS_INFO_STREAM("AI = "<<acceleration_PID.m_ki);
    ROS_INFO_STREAM("AD = "<<acceleration_PID.m_kd);
    velocity_error = vehicle_state.velocity_error;
    ROS_INFO_STREAM("velocity_error="<<vehicle_state.velocity_error);
    // ROS_INFO_STREAM("distance_error="<<vehicle_state.distance_error);
    // ROS_INFO_STREAM("heading_angle_error="<<vehicle_state.heading_error);
    velocity_PID.control_position(velocity_error, acc_compensation,Ts);
    //velocity_PID.control_increment(velocity_error, acc_compensation,Ts);
    slope_compensation = vehicle_state.G*sin(vehicle_state.slope);
    ROS_INFO_STREAM("acceleration compensation="<<acc_compensation);
    //acc_error=trajectory_analyzer.goal_state.a+acc_compensation+slope_compensation-vehicle_state.acceleration;
    acc_error=trajectory_analyzer.goal_state.a+acc_compensation-vehicle_state.acceleration;
    ROS_INFO_STREAM("goal acceleration ="<<trajectory_analyzer.goal_state.a);
    ROS_INFO_STREAM("actual acceleration ="<<vehicle_state.acceleration);
    ROS_INFO_STREAM("acceleration error="<<acc_error);
    acceleration_PID.control_position(acc_error,torque_cmd,Ts);
    //acceleration_PID.control_increment(acc_error,torque_cmd,Ts);
    ROS_INFO_STREAM("torque_cmd="<<torque_cmd);
    control_cmd.throttle=torque_cmd;

    ROS_INFO("Compute longitudinal command successfully!");

    status.status = "OK";
    return status;
}