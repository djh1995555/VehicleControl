#include<control/Controller/StanleyController.h>

Status StanleyController::Init(const ControlConf &control_conf)
{
    name = "PurePursuit controller";
    Ts = control_conf.conf_param.Ts;
    wheel_base=control_conf.conf_param.wheel_base;
    velocity_gain=control_conf.conf_param.velocity_gain;
    
    cout<<name<<" is initialized successfully!"<<endl;
    status.status = "OK";
    return status;

}
void StanleyController::TransformToFrontAxle(double x,double y)
{
    front_axle_x = x+wheel_base*cos(current_heading);
    front_axle_y= y +wheel_base*sin(current_heading);
}
Status StanleyController::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const  VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    ROS_INFO("Use Pure Pursuit");
    double x = vehicle_state.movement_state.pose.position.x;
    double y = vehicle_state.movement_state.pose.position.y;
    goal_state = trajectory_analyzer.goal_state;
    current_heading = vehicle_state.heading_angle;
    current_velocity=vehicle_state.current_velocity;
    TransformToFrontAxle(x,y);
    double goal_id = trajectory_analyzer.MatchPointByPositionForStanley(front_axle_x,front_axle_y);
    goal_state = trajectory_analyzer.trajectory_info[goal_id];
    // ROS_INFO_STREAM("front_axle_x = "<<front_axle_x);
    // ROS_INFO_STREAM("front_axle_y = "<<front_axle_y);
    // ROS_INFO_STREAM("goal_x = "<<goal_state.x);
    // ROS_INFO_STREAM("goal_y = "<<goal_state.y);
   
    // original method
/*     double distance;
    double temp = atan2(front_axle_y-goal_state.y,front_axle_x-goal_state.x);
    double angle_diff = goal_state.theta-temp;
    if(angle_diff>pi)
    {
        angle_diff=angle_diff-2*pi;
    }
    else if(angle_diff<-pi)
    {
        angle_diff=angle_diff+2*pi;
    }
    if(angle_diff>0)
    {
        distance = trajectory_analyzer.ComputeDist(goal_state,front_axle_x,front_axle_y);
    }
    else
    {
        distance = -trajectory_analyzer.ComputeDist(goal_state,front_axle_x,front_axle_y);
    }
    double d_theta = goal_state.theta-current_heading;
    if(d_theta>pi)
    {
        d_theta=d_theta-2*pi;
    }
    else if(d_theta<-pi)
    {
        d_theta=d_theta+2*pi;
    }
    steering_angle = d_theta +atan2(velocity_gain*distance,current_velocity);
    ROS_INFO_STREAM("heading error = "<<goal_state.theta-current_heading);  
    ROS_INFO_STREAM("current_velocity = "<<current_velocity);
    ROS_INFO_STREAM("distance = "<<distance);
    ROS_INFO_STREAM("angle = "<<atan2(velocity_gain*distance,current_velocity)); */

    
    // modified method, in this case we do not need to consider the goal point is in the right side or left side
    double d = current_velocity/velocity_gain;
    double d_x=goal_state.x+d*cos(goal_state.theta);
    double d_y=goal_state.y+d*sin(goal_state.theta);
    steering_angle = atan2(d_y-front_axle_y,d_x-front_axle_x)-current_heading;
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