#include <control/VehicleState.h>

Status VehicleState::Init(const ControlConf &control_conf)
{
    Ts=control_conf.conf_param.Ts;
    slope_threshold=control_conf.conf_param.slope_threshold;
    pitch_deque_length=control_conf.conf_param.pitch_deque_length;
    filter_method=control_conf.conf_param.filter_method;

    G=control_conf.conf_param.G;
    mass=control_conf.conf_param.mass;
    friction_coefficient=control_conf.conf_param.friction_coefficient;
    wheel_base=control_conf.conf_param.wheel_base;
    front_wheel_base=control_conf.conf_param.front_wheel_base;
    rear_wheel_base=control_conf.conf_param.rear_wheel_base;
    front_track=control_conf.conf_param.front_track;
    rear_track=control_conf.conf_param.rear_track;
    wheel_radius=control_conf.conf_param.wheel_radius;
    Ix=control_conf.conf_param.Ix;
    Iy=control_conf.conf_param.Iy;
    Iz=control_conf.conf_param.Iz;
    brake_distance_coefficient=control_conf.conf_param.brake_distance_coefficient;
    chassis_aero_force_gain=control_conf.conf_param.chassis_aero_force_gain;
    max_front_torque=control_conf.conf_param.max_front_torque;
    max_back_torque=control_conf.conf_param.max_back_torque;
    max_front_brake_torque=control_conf.conf_param.max_front_brake_torque;
    max_back_brake_torque=control_conf.conf_param.max_back_brake_torque;
    max_speed=control_conf.conf_param.max_speed;
    max_steer=control_conf.conf_param.max_steer;
    gas_efficiency=control_conf.conf_param.gas_efficiency;
    battery_charge_watt_hours=control_conf.conf_param.battery_charge_watt_hours;
    battery_discharge_watt_hours=control_conf.conf_param.battery_discharge_watt_hours;
    flwheel_steering_p_gain=control_conf.conf_param.flwheel_steering_p_gain;
    frwheel_steering_p_gain=control_conf.conf_param.frwheel_steering_p_gain;
    flwheel_steering_i_gain=control_conf.conf_param.flwheel_steering_i_gain;
    frwheel_steering_i_gain=control_conf.conf_param.frwheel_steering_i_gain;
    flwheel_steering_d_gain=control_conf.conf_param.flwheel_steering_d_gain;
    frwheel_steering_d_gain=control_conf.conf_param.frwheel_steering_d_gain;

    for(int i=0;i<pitch_deque_length;++i)
    {
        pitch_deque.push_back(0);
    }

    status.status = "OK";
    return status;
}
void VehicleState::ComputeVelocityError(const prius_msgs::My_Trajectory_Point &goal_state)
{
    heading_error = goal_state.theta-heading_angle;
    //velocity_error = goal_state.v-current_velocity*cos(heading_error);
    velocity_error = goal_state.v-current_velocity;
    ROS_INFO_STREAM("current velocity = "<<current_velocity);
    ROS_INFO_STREAM("goal velocity = "<<goal_state.v);
    ROS_INFO_STREAM("heading_error = "<<heading_error);
}
Status VehicleState::GetVehicleStateFromLocalization(const nav_msgs::Odometry &localization)
{
    ++cycle_num;
    movement_state.pose =  localization.pose.pose;
    ros::param::set("vehicle_x", movement_state.pose.position.x);
    ros::param::set("vehicle_y", movement_state.pose.position.y);
    movement_state.twist =  localization.twist.twist;
    ComputeVelocity();
    //ROS_INFO_STREAM("Current velocity(from chassis) is "<<info_from_chassis.longitudinal_data.vel_from_localization);
    ComputeAcc();
    //ROS_INFO_STREAM("Current acceleration(from chassis) is "<<info_from_chassis.longitudinal_data.acceleration);
    float x = movement_state.pose.orientation.x;
    float y = movement_state.pose.orientation.y;
    float z = movement_state.pose.orientation.z;
    float w = movement_state.pose.orientation.w;
    ComputeEular(x,y,z,w);
    ComputeHeadingAngle();
    //ROS_INFO_STREAM("Current heading(from chassis) is "<<info_from_chassis.lateral_data.heading_angle);

    ComputeSlope();
    status.status = "OK";
    return status;
}
Status VehicleState::GetVehicleStateFromCarsim(const nav_msgs::Odometry &carsim_feedback)
{
    movement_state.pose =  carsim_feedback.pose.pose;
    ros::param::set("vehicle_x", movement_state.pose.position.x);
    ros::param::set("vehicle_y", movement_state.pose.position.y);
    movement_state.twist =  carsim_feedback.twist.twist;
    ComputeVelocity();
    //ROS_INFO_STREAM("Current velocity(from chassis) is "<<info_from_chassis.longitudinal_data.vel_from_localization);
    ComputeAcc();
    //ROS_INFO_STREAM("Current acceleration(from chassis) is "<<info_from_chassis.longitudinal_data.acceleration);

    pose_angle[0]=carsim_feedback.pose.pose.orientation.x*PI/180;
    pose_angle[1]=carsim_feedback.pose.pose.orientation.y*PI/180;
    pose_angle[2]=carsim_feedback.pose.pose.orientation.z*PI/180;
    ComputeHeadingAngle();
    ComputeSlope();
    status.status = "OK";
    return status;
}
Status VehicleState::GetVehicleStateFromChassis(const prius_msgs::VehicleInfo &current_state)
{
    info_from_chassis = current_state;
    status.status = "OK";
    return status;
}
void VehicleState::CountErrors()
{
    last_lateral_distance_error = lateral_distance_error;
    if(abs(lateral_distance_error-last_lateral_distance_error)>distance_difference_threshold)
    {
        lateral_distance_error=last_lateral_distance_error;
    }
    max_lateral_distance_error=abs(lateral_distance_error)>abs(max_lateral_distance_error)?lateral_distance_error:max_lateral_distance_error;

    last_velocity_error = velocity_error;
    if(abs(velocity_error-last_velocity_error)>velocity_difference_threshold)
    {
        velocity_error=last_velocity_error;
    }
    max_velocity_error=abs(velocity_error)>abs(max_velocity_error)?velocity_error:max_velocity_error;

    last_heading_error = heading_error;
    if(abs(heading_error-last_heading_error)>heading_difference_threshold)
    {
        heading_error=last_heading_error;
    }
    if(heading_error>PI)
    {
        heading_error = heading_error-2*PI;
    }
    else if(heading_error<-PI)
    {
        heading_error = heading_error+2*PI;
    }
    max_heading_error=abs(heading_error)>abs(max_heading_error)?heading_error:max_heading_error;
    if(state!="STOP")
    {
        average_lateral_distance_error = average_lateral_distance_error+(abs(lateral_distance_error)-average_lateral_distance_error)/cycle_num;  
        average_velocity_error = average_velocity_error+(abs(velocity_error)-average_velocity_error)/cycle_num;
        average_heading_error = average_heading_error+(abs(heading_error)-average_heading_error)/cycle_num;
    }

}
void VehicleState::GetAllData(double goal_id,double preview_id,const ControlConf &control_conf,const double computational_time,const double average_time,const double max_time)
{
    CountErrors();
    vehicle_info.computational_time = computational_time;
    vehicle_info.average_computational_time = average_time;
    vehicle_info.max_computational_time = max_time;

    vehicle_info.lateral_distance_error = lateral_distance_error;
    vehicle_info.average_lateral_distance_error = average_lateral_distance_error;
    vehicle_info.max_lateral_distance_error = max_lateral_distance_error;

    vehicle_info.velocity_error = velocity_error;
    vehicle_info.average_velocity_error = average_velocity_error;
    vehicle_info.max_velocity_error = max_velocity_error;

    vehicle_info.heading_error = heading_error;
    vehicle_info.average_heading_error = average_heading_error;
    vehicle_info.max_heading_error = max_heading_error;

    vehicle_info.goal_id=goal_id;
    vehicle_info.preview_id=preview_id;
    vehicle_info.slope =slope;
    vehicle_info.trajectory_point.x=movement_state.pose.position.x;
    vehicle_info.trajectory_point.y=movement_state.pose.position.y;
    vehicle_info.trajectory_point.v=current_velocity;
    vehicle_info.trajectory_point.theta = heading_angle;
    vehicle_info.trajectory_point.relative_time=ros::Time::now().toSec();
    vehicle_info.trajectory_point.a=acceleration;
    vehicle_info.vehicle_info = info_from_chassis;
}

void VehicleState::ComputeDistanceFromDestination(const prius_msgs::My_Trajectory_Point destination)
{
    double x = movement_state.pose.position.x;
    double y = movement_state.pose.position.y;
    this->distance_from_destination = sqrt((destination.x-x)*(destination.x-x)+(destination.y-y)*(destination.y-y));
    //ROS_INFO_STREAM("Current distance from destination(from GPS) is "<<distance_from_destination);
}
void VehicleState::ComputeVelocity()
{
    current_velocity= sqrt(movement_state.twist.linear.x*movement_state.twist.linear.x+movement_state.twist.linear.y*movement_state.twist.linear.y);
    ros::param::set("vehicle_v", current_velocity);
    //ROS_INFO_STREAM("Current velocity(from GPS) is "<<current_velocity);
}


void VehicleState::ComputeAcc()
{
    acceleration= (current_velocity-previous_velocity)/Ts;
    previous_velocity = current_velocity;
    ros::param::set("current_a", current_velocity);
    //ROS_INFO_STREAM("Current acceleration(from GPS) is "<<acceleration);
}
void VehicleState::ComputeEular(double x,double y,double z,double w)
{
    double roll = atan2(2 * (y*z + w*x), w*w - x*x - y*y + z*z);
    double pitch = asin(-2 * (x*z - w*y));
    double yaw = atan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z);
    pose_angle[0]=roll;
    pose_angle[1]=yaw;
    pose_angle[2]=pitch;

}
void VehicleState::ComputeHeadingAngle()
{
    heading_angle = pose_angle[1];
    //ROS_INFO_STREAM("Current heading angle(from GPS) is "<<heading_angle);
}

double VehicleState::Filter(const deque<double> pitch_deque,double method)
{
    double slope;
    switch(int(method))
    {case 1:
        slope = MedianFilter(pitch_deque);
        break;
    case 2:
        slope = SlideWindowMeanFilter(pitch_deque);
        break;
    }
    return slope;
}
double VehicleState::MedianFilter(deque<double> pitch_deque)
{
    sort(pitch_deque.begin(),pitch_deque.end());
    return pitch_deque[int(pitch_deque_length)/2];
}
double VehicleState::SlideWindowMeanFilter(deque<double> pitch_deque)
{
    double sum;
    for(int i=0;i<pitch_deque_length;++i)
    {
        sum +=pitch_deque[i];
    }
    return sum/pitch_deque_length;
}
    
void VehicleState::ComputeSlope()
{
    pitch_deque.pop_front();
    pitch_deque.push_back(pose_angle[2]);
    slope = -Filter(pitch_deque,filter_method)*180/PI;
    if(abs(slope)<slope_threshold)
    {
        slope=0;
    }
}