#include<control/Controller/RearWheelFeedbackController.h>

Status RearWheelFeedbackController::Init(const ControlConf &control_conf)
{
    name = "PurePursuit controller";
    Ts = control_conf.conf_param.Ts;
    yaw_error_gain = control_conf.conf_param.yaw_error_gain;
    lateral_error_gain=control_conf.conf_param.lateral_error_gain;
    wheel_base=control_conf.conf_param.wheel_base;
    
    cout<<name<<" is initialized successfully!"<<endl;
    status.status = "OK";
    return status;

}

Status RearWheelFeedbackController::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    ROS_INFO("Use Rear Wheel Feedback Controller");
    double x = vehicle_state.movement_state.pose.position.x;
    double y = vehicle_state.movement_state.pose.position.y;
    goal_state=trajectory_analyzer.goal_state;

/*     trajectory_analyzer.GetPreviewPoint(preview_length);
    goal_state=trajectory_analyzer.preview_state; */


    current_velocity=vehicle_state.current_velocity;

    double distance= trajectory_analyzer.ComputeDist(goal_state,x,y);
    double dx=x-goal_state.x;
    double dy=y-goal_state.y;
    // ROS_INFO_STREAM("dx = "<<dx);
    // ROS_INFO_STREAM("dy = "<<dy);
    lateral_error = dy*cos(goal_state.theta)-dx*sin(goal_state.theta);
    //lateral_error=distance;
    ROS_INFO_STREAM("lateral_error = "<<lateral_error);
    

    double omega;
    heading_error = vehicle_state.heading_angle-goal_state.theta;
    if(heading_error>pi)
    {
        heading_error=heading_error-2*pi;
    }
    else if(heading_error<-pi)
    {
        heading_error=heading_error+2*pi;
    }
    double a;

    a=sin(heading_error)/heading_error;

    omega = current_velocity * goal_state.kappa*cos(heading_error)/(1-goal_state.kappa*lateral_error)
    -yaw_error_gain*abs(current_velocity)*heading_error-lateral_error_gain*current_velocity*lateral_error*a;
    // ROS_INFO_STREAM("omega = "<<omega);
    // ROS_INFO_STREAM("item1 = "<<goal_state.kappa*cos(heading_error)/(1-goal_state.kappa*lateral_error));
    // ROS_INFO_STREAM("item2 = "<<yaw_error_gain*heading_error);
    // ROS_INFO_STREAM("item3 = "<<lateral_error_gain*lateral_error*a);
    // ROS_INFO_STREAM("item4 = "<<a);

    // ROS_INFO_STREAM("current_heading = "<<vehicle_state.heading_angle);
    // ROS_INFO_STREAM("goal_heading = "<<goal_state.theta);
    // ROS_INFO_STREAM("heading_error = "<<heading_error);

    steering_angle = atan2(wheel_base*omega/current_velocity,1.0);
    if(steering_angle>pi)
    {
        steering_angle=steering_angle-2*pi;
    }
    else if(steering_angle<-pi)
    {
        steering_angle=steering_angle+2*pi;
    }
    control_cmd.steer = steering_angle;
    ROS_INFO("Compute lateral command successfully!");
    status.status = "OK";
    return status;
}