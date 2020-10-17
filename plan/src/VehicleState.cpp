#include <planning/VehicleState.h>

Status VehicleState::Init(const PlanningConf &planning_conf)
{
    ROS_INFO("vehicle state is initialized!");
    Ts = planning_conf.planning_param.Ts;
    G=planning_conf.planning_param.G;
    mass=planning_conf.planning_param.mass;
    friction_coefficient=planning_conf.planning_param.friction_coefficient;
    wheel_base=planning_conf.planning_param.wheel_base;
    front_wheel_base=planning_conf.planning_param.front_wheel_base;
    rear_wheel_base=planning_conf.planning_param.rear_wheel_base;
    front_track=planning_conf.planning_param.front_track;
    rear_track=planning_conf.planning_param.rear_track;
    wheel_radius=planning_conf.planning_param.wheel_radius;
    Ix=planning_conf.planning_param.Ix;
    Iy=planning_conf.planning_param.Iy;
    Iz=planning_conf.planning_param.Iz;
    brake_distance_coefficient=planning_conf.planning_param.brake_distance_coefficient;
    chassis_aero_force_gain=planning_conf.planning_param.chassis_aero_force_gain;
    max_front_torque=planning_conf.planning_param.max_front_torque;
    max_back_torque=planning_conf.planning_param.max_back_torque;
    max_front_brake_torque=planning_conf.planning_param.max_front_brake_torque;
    max_back_brake_torque=planning_conf.planning_param.max_back_brake_torque;
    max_speed=planning_conf.planning_param.max_speed;
    max_steer=planning_conf.planning_param.max_steer;
    gas_efficiency=planning_conf.planning_param.gas_efficiency;
    battery_charge_watt_hours=planning_conf.planning_param.battery_charge_watt_hours;
    battery_discharge_watt_hours=planning_conf.planning_param.battery_discharge_watt_hours;
    flwheel_steering_p_gain=planning_conf.planning_param.flwheel_steering_p_gain;
    frwheel_steering_p_gain=planning_conf.planning_param.frwheel_steering_p_gain;
    flwheel_steering_i_gain=planning_conf.planning_param.flwheel_steering_i_gain;
    frwheel_steering_i_gain=planning_conf.planning_param.frwheel_steering_i_gain;
    flwheel_steering_d_gain=planning_conf.planning_param.flwheel_steering_d_gain;
    frwheel_steering_d_gain=planning_conf.planning_param.frwheel_steering_d_gain;
    speed_deque_length = planning_conf.planning_param.speed_deque_length;

    BoundingBox box(planning_conf.planning_param.length,planning_conf.planning_param.width,planning_conf.planning_param.height);
    VehicleBox = box;
    VehicleBox.GetDiagonal();

    for(int i=0;i<speed_deque_length;++i)
    {
        speed_deque.push_back(0);
    }

    status.status = "OK";
    return status;
}

void VehicleState::ComputeVelocityError(const prius_msgs::My_Trajectory_Point &goal_state)
{
    heading_error = goal_state.theta-heading_angle;
    velocity_error = goal_state.v-current_velocity*cos(heading_error);
}
Status VehicleState::GetVehicleStateFromLocalization(const nav_msgs::Odometry &localization)
{
    movement_state.pose =  localization.pose.pose;
    ros::param::set("vehicle_x", movement_state.pose.position.x);
    ros::param::set("vehicle_y", movement_state.pose.position.y);
    movement_state.twist =  localization.twist.twist;
    ComputeVelocity();
    ComputeAcc();
    safe_distance = current_velocity*current_velocity/(2*max_deceleration);
    float x = movement_state.pose.orientation.x;
    float y = movement_state.pose.orientation.y;
    float z = movement_state.pose.orientation.z;
    float w = movement_state.pose.orientation.w;
    ComputeEular(x,y,z,w);
    ComputeHeadingAngle();
    status.status = "OK";
    return status;
}

Status VehicleState::GetVehicleStateFromChassis(const prius_msgs::VehicleInfo &current_state)
{
    info_from_chassis = current_state;
    status.status = "OK";
    return status;
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
    //ROS_INFO_STREAM("Current velocity(from GPS) is "<<current_velocity);
}


void VehicleState::ComputeAcc()
{
    speed_deque.pop_front();
    speed_deque.push_back(current_velocity);
    acceleration= (speed_deque.back()-speed_deque.front())/(Ts*speed_deque_length);
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
    VehicleBox.SetHeading(heading_angle);
    //ROS_INFO_STREAM("Current heading angle(from GPS) is "<<heading_angle);
}


