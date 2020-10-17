#include<control/Controller/PurePursuit.h>

Status PurePursuit::Init(const ControlConf &control_conf)
{
    name = "PurePursuit controller";
    Ts = control_conf.conf_param.Ts;
    pure_pursuit_preview_length = control_conf.conf_param.pure_pursuit_preview_length;
    
    cout<<name<<" is initialized successfully!"<<endl;
    status.status = "OK";
    return status;

}
Status PurePursuit::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, const VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    //ROS_INFO("Use PurePsuit");
    double x = vehicle_state.movement_state.pose.position.x;
    double y = vehicle_state.movement_state.pose.position.y;
    trajectory_analyzer.GetPreviewPoint(pure_pursuit_preview_length);
    preview_state = trajectory_analyzer.preview_state;
    //ROS_INFO_STREAM("pure_pursuit_preview_length="<<pure_pursuit_preview_length);
    ROS_INFO_STREAM("preview state x="<<preview_state.x);
    ROS_INFO_STREAM("preview state y="<<preview_state.y); 
    double heading_angle = vehicle_state.heading_angle;

    alpha = atan2(preview_state.y-y, preview_state.x-x)-(heading_angle);
    ROS_INFO_STREAM("Alpha="<<alpha);
    preview_distance = trajectory_analyzer.ComputeDist(preview_state,x,y);
    steering_angle = atan(2*vehicle_state.wheel_base*sin(alpha)/preview_distance);
    
    control_cmd.steer = steering_angle;
    ROS_INFO("Compute lateral command successfully!");
    status.status = "OK";
    return status;
}
