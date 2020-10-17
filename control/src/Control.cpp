#include<control/Control.h>

int Control::Spin()
{
    Status init_status = Init();
    if(init_status.status != "OK")
    {
        ROS_INFO("Control Init Failed!");
        return -1;
    }

    Status start_status = Start();
    if(start_status.status != "OK")
    {
        ROS_INFO("Control Start Failed!");
        return -2;
    }

    ros::spin();
/*     ros::waitForShutdown();
    Stop();
    ROS_INFO_STREAM(name<<" exited!"); */
    return 0;
}
Status Control::Init()
{
    init_time = (ros::Time::now()).toSec();
    ROS_INFO("Control Init...");
    Status read_status = control_conf.ReadControlConf(CONTROL_CONF_FILE_NAME);
    node.getParam("simulation_platform", simulation_platform);

    int controllers_num;
    string longitudinal_controller;
    string lateral_controller;
    string centralized_controller;
    node.param<int>("controllers_num", controllers_num, 0);
    node.param<std::string>("longitudinal_controller", longitudinal_controller, "xx");
    node.param<std::string>("lateral_controller", lateral_controller, "xx");
    node.param<std::string>("centralized_controller", centralized_controller, "xx");
    node.setParam("controllers_num",int(control_conf.conf_param.controller_num));
    node.setParam("longitudinal_controller",control_conf.LONGITUDINAL_CONTROLLER);
    node.setParam("lateral_controller", control_conf.LATERAL_CONTROLLER);
    node.setParam("centralized_controller",control_conf.CENTRALIZED_CONTROLLER);
    if(read_status.status != "OK")
    {
        ROS_INFO_STREAM("Unable to load control conf file "<<CONTROL_CONF_FILE_NAME);
        status.status = "Failed";
        return status;
    }
    ROS_INFO_STREAM("Control conf file:"<<CONTROL_CONF_FILE_NAME<<" had beed loaded!");
    


    Status vehicle_instance_status = vehicle_state.Init(control_conf);
    if(vehicle_instance_status.status != "OK")
    {
        ROS_INFO("Vehicle instance init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }
    ROS_INFO("Vehicle instance init successfully!");

    Status analyzer_status = trajectory_analyzer.Init(control_conf);
    if(analyzer_status.status != "OK")
    {
        ROS_INFO("Trajectory analyzer init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }

    ROS_INFO("Trajectory analyzer init successfully!");
    Status agent_status = controller_agent.Init(control_conf);
    if(agent_status.status != "OK")
    {
        ROS_INFO("Controller agent init failed! Stopping...");
        status.status = "Failed";
        return status;       
    }

    ROS_INFO("Controller agent init successfully!");
    status.status = "OK";
    return status;
}

Status Control::Start()
{
    ROS_INFO("Control resetting vehicle state, sleeping for 1000ms...");
    ros::Duration(1).sleep();
    vehicle_state.state = "WORKING!";
    ROS_INFO_STREAM("Vehicle current state is"<<vehicle_state.state);

    
    vehicle_info_pub=node.advertise<prius_msgs::Augmented_My_Trajectory_Point>("/prius/vehicle_info", 10, true);
 
    trajectory_sub=node.subscribe("/ontime_trajectory",10, &Control::TrajectoryCallback,this);
    //trajectory_sub=node.subscribe("/global_trajectory",10,&Control::TrajectoryCallback,this);


    if(simulation_platform=="GAZEBO")
    {
        // used for prius model
        command_pub=node.advertise<nox_msgs::SignalArray>("/chassis_signals", 10, true);
        localization_sub=node.subscribe("/base_pose_ground_truth",10,&Control::LocalizationCallback,this);
        ROS_INFO_STREAM("Current simulation platform  is"<<simulation_platform);
        chassis_info_sub=node.subscribe("/prius/chassis_info",10,&Control::ChassisInfoCallback,this);
        chassis_state_sub=node.subscribe("chassis_states",10,&Control::ChassisStateCallback,this);

    }
    else if(simulation_platform=="CARSIM")
    {
        // used for CARSIM simulation
        carsim_sub=node.subscribe("/carsim_feedback",10, &Control::CARSIMCallback,this);
        carsim_pub=node.advertise<geometry_msgs::Point>("/carsim_control", 10, true);
        ROS_INFO_STREAM("Current simulation platform  is"<<simulation_platform);
    }
    status.status = "OK";
    return status;
}
void Control::Stop()
{
    // TO DO
}



void Control::TrajectoryCallback(const prius_msgs::My_Trajectory &trajectory)
{
    ++received_trajectory_num;
    trajectory_analyzer.goal_id = 0;
    trajectory_analyzer.trajectory_info.clear();
    //ROS_INFO("read local trajectory");
    Status trajectory_status =trajectory_analyzer.ReadTrajectory(trajectory);
    //ROS_INFO("read finish");
    trajectory_analyzer.ESTOP = trajectory.ESTOP;
    if(trajectory_status.status != "OK")
    {
        ROS_INFO("Read trajectory failed!");
        return;
    }
    trajectory_analyzer.PrintTrajectory();

    if(received_trajectory_num==1)
    {
        string trajectory_velocity;
        node.getParam("/trajectory_velocity",trajectory_velocity);
        string lateral_controller_name = controller_agent.lateral_controller;
        replace(lateral_controller_name.begin(),lateral_controller_name.end(),' ','_');
        string simulation_result="/home/djh/ControlModule/src/control/plot_py/simulation_result_"+lateral_controller_name+'_'+trajectory_velocity+".txt";
        ROS_INFO_STREAM("simulation_result is "<<simulation_result);
        ofs.open(simulation_result,ios::out);
        if(!ofs.is_open())
        {
            ROS_INFO("打开文件失败！");
        }
        ROS_INFO("ready to write in!");
    }


}

void Control::LocalizationCallback(const nav_msgs::Odometry &localization)
{   
    // if(count<LOCALIZATION_FREQUENCY/frequency)
    // {
    //     ++count;
        
    //     return;
    // }
    //ROS_INFO_STREAM("(control) count is "<< count);
    Status localization_status=vehicle_state.GetVehicleStateFromLocalization(localization);
    if(localization_status.status != "OK")
    {
        ROS_INFO("Get vehicle state failed!");
        return;
    }
    //ROS_INFO_STREAM("(control) current x ="<<vehicle_state.movement_state.pose.position.x );
    //ROS_INFO_STREAM("(control) current y ="<<vehicle_state.movement_state.pose.position.y );
    vehicle_state.ComputeDistanceFromDestination(trajectory_analyzer.destination);
    //ROS_INFO_STREAM("(control) destination x ="<<trajectory_analyzer.destination.x);
    //ROS_INFO_STREAM("(control) destination y ="<<trajectory_analyzer.destination.y);

    bool is_reach_destination = abs(trajectory_analyzer.goal_state.x-trajectory_analyzer.destination.x)<0.01
    &&abs(trajectory_analyzer.goal_state.y-trajectory_analyzer.destination.y)<0.01;

    if (trajectory_analyzer.trajectory_info.empty()||is_reach_destination)
    {
        control_cmd.throttle = 0.0;
        control_cmd.brake = 1.0;
        vehicle_state.state = "STOP";
        if(trajectory_analyzer.trajectory_info.empty())
        {
            ROS_INFO("local trajectory is empty!");
        }
        else
        {
            ROS_INFO("reach destination!");
        }
        trajectory_analyzer.trajectory_info.clear();
        ROS_INFO("Vehicle will stop!");
    }
    else if(trajectory_analyzer.ESTOP)
    {
        control_cmd.throttle = 0.0;
        control_cmd.brake = 1.0;
        ROS_INFO("obstacle is detected.");
    }
    else
    {
        vehicle_state.state = "WORKING";
        if(timestamp_num==0)
        {
            start_time = (ros::Time::now()).toNSec()/1000000;
        }
        Status goal_status=trajectory_analyzer.MatchPointByPosition(vehicle_state);
        if(goal_status.status != "OK")
        {
            ROS_INFO("get the nearest point failed!");
            return;
        }

        //ROS_INFO("Find the nearest point!");
        //ROS_INFO_STREAM("goal x="<<trajectory_analyzer.goal_state.x );
        //ROS_INFO_STREAM("goal y="<<trajectory_analyzer.goal_state.y );
        if(trajectory_analyzer.goal_state.gear == 1)
        {
            control_cmd.shift_gears = control_cmd.FORWARD;
        }
        else if(trajectory_analyzer.goal_state.gear == -1)
        {
            control_cmd.shift_gears = control_cmd.REVERSE;
            vehicle_state.heading_angle -= PI;
        }
        //ROS_INFO_STREAM("direction is "<< trajectory_analyzer.goal_state.gear);
        vehicle_state.ComputeVelocityError(trajectory_analyzer.goal_state);


        Status cmd_status=controller_agent.ComputeControlCmd(trajectory_analyzer,vehicle_state, control_cmd);
  

        if(cmd_status.status != "OK")
        {
            ROS_INFO("Compute control command failed!");
            trajectory_analyzer.trajectory_info.clear();
            control_cmd.throttle = 0.0;
            control_cmd.brake = 1.0;
            ROS_INFO("Vehicle will stop!");
            return;
        }
        ++timestamp_num;
        ++control_cycle_num;
        if(timestamp_num == timestamp_range)
        {
            last_computational_time = computational_time;
            end_time = (ros::Time::now()).toNSec()/1000000;
            computational_time = (end_time-start_time)/timestamp_num;
            if(abs(computational_time-last_computational_time)>time_difference_threshold)
            {
                computational_time=last_computational_time;
            }
            average_computational_time = average_computational_time+(computational_time-average_computational_time)/control_cycle_num;
            max_computational_time = computational_time>max_computational_time?computational_time:max_computational_time;

            // ROS_INFO_STREAM("computational time of ControlModule = "<<computational_time<<" ms."); 
            // ROS_INFO_STREAM("average computational time of ControlModule = "<<average_computational_time<<" ms."); 
            // ROS_INFO_STREAM("max computational time of ControlModule = "<<max_computational_time<<" ms."); 
            timestamp_num =0.0;
        }  
    }

    defines::Panel panel;
    // longitudinal
    panel.Throttle.Set(control_cmd.throttle);
    panel.Throttle.Enable();
    panel.Brake.Set(control_cmd.brake);
    panel.Brake.Enable();
    
    panel.Handbrake.Set(0);
    panel.Handbrake.Enable();
    if(control_cmd.shift_gears == control_cmd.FORWARD)
    {
        panel.Gear.Set(defines::GearState::P);
    }
    else if(control_cmd.shift_gears == control_cmd.REVERSE)
    {
        panel.Gear.Set(defines::GearState::R);
    }
    else if(control_cmd.shift_gears == control_cmd.NEUTRAL)
    {
        panel.Gear.Set(defines::GearState::N);
    }   
    panel.Gear.Enable();
    // lateral
    panel.Steer.Set(control_cmd.steer);
    panel.Steer.Enable();

    command_pub.publish(panel.ToMsgs());
    //command_pub.publish(control_cmd);

    //ROS_INFO("Control command is published!");
    vehicle_state.GetAllData(trajectory_analyzer.goal_id,trajectory_analyzer.preview_id,control_conf,computational_time,average_computational_time,max_computational_time);
    //ROS_INFO("Data is gathered");
    vehicle_info_pub.publish(vehicle_state.vehicle_info);
    //ROS_INFO("Vehicle info is published!");
    WriteInDebug(ofs,vehicle_state.vehicle_info,trajectory_analyzer,controller_agent);
    //ROS_INFO("Write in debug successfully!");
    return;
}
void Control::CARSIMCallback(const nav_msgs::Odometry &carsim_feedback)
{
    Status localization_status=vehicle_state.GetVehicleStateFromCarsim(carsim_feedback);
    if(localization_status.status != "OK")
    {
        ROS_INFO("Get vehicle state failed!");
        return;
    }
    // ROS_INFO_STREAM("current x ="<<vehicle_state.movement_state.pose.position.x );
    // ROS_INFO_STREAM("current y ="<<vehicle_state.movement_state.pose.position.y );
    vehicle_state.ComputeDistanceFromDestination(trajectory_analyzer.destination);
    // ROS_INFO_STREAM("destination x ="<<trajectory_analyzer.destination.x);
    // ROS_INFO_STREAM("destination y ="<<trajectory_analyzer.destination.y);
    //vehicle_state.ComputeBrakeDistanceAhead();
    // ROS_INFO_STREAM("Brake distance ahead is"<<vehicle_state.brake_distance_ahead);
    // ROS_INFO_STREAM("Distance from destination is "<<vehicle_state.distance_from_destination);
    //if (trajectory_analyzer.trajectory_info.empty()||vehicle_state.distance_from_destination<=vehicle_state.brake_distance_ahead)
    if (trajectory_analyzer.trajectory_info.empty())
    {
        
        
        control_cmd.throttle = 0.0;
        control_cmd.brake = 1.0;
        if(trajectory_analyzer.trajectory_info.empty())
        {
            ROS_INFO("trajectory is empty!");
        }
        else
        {
            ROS_INFO("reach destination!");
        }
        trajectory_analyzer.trajectory_info.clear();
        ROS_INFO("Vehicle will stop!");
    }
    else
    {
        Status goal_status=trajectory_analyzer.MatchPointByPosition(vehicle_state);
        if(goal_status.status != "OK")
        {
            ROS_INFO("get the nearest point failed!");
            return;
        }

        // ROS_INFO("Find the nearest point!");
        // ROS_INFO_STREAM("goal x="<<trajectory_analyzer.goal_state.x );
        // ROS_INFO_STREAM("goal y="<<trajectory_analyzer.goal_state.y );
        
        vehicle_state.ComputeVelocityError(trajectory_analyzer.goal_state);

        Status cmd_status=controller_agent.ComputeControlCmd(trajectory_analyzer,vehicle_state, control_cmd);
        if(cmd_status.status != "OK")
        {
            ROS_INFO("Compute control command failed!");
            trajectory_analyzer.trajectory_info.clear();
            control_cmd.throttle = 0.0;
            control_cmd.brake = 1.0;
            ROS_INFO("Vehicle will stop!");
            return;
        }
    }  
    geometry_msgs::Point carsim_cmd;
    carsim_cmd.x=control_cmd.throttle;
    carsim_cmd.y=control_cmd.brake;
    carsim_cmd.z=control_cmd.steer;
    carsim_pub.publish(carsim_cmd);
    //ROS_INFO("Control command is published!");
    vehicle_state.GetAllData(trajectory_analyzer.goal_id,trajectory_analyzer.preview_id,control_conf,computational_time,average_computational_time,max_computational_time);
    ROS_INFO("Data is gathered");
    vehicle_info_pub.publish(vehicle_state.vehicle_info);
    ROS_INFO("Vehicle info is published!");
    WriteInDebug(ofs,vehicle_state.vehicle_info,trajectory_analyzer,controller_agent);
    
    return;
    
}
void Control::ChassisInfoCallback(const prius_msgs::VehicleInfo &chassis_info)
{
    Status chassis_status=vehicle_state.GetVehicleStateFromChassis(chassis_info);
    if(chassis_status.status != "OK")
    {
        ROS_INFO("Get vehicle info failed!");
        return;
    }
    //ROS_INFO("Get vehicle info successfully!");    
}


void Control::ChassisStateCallback(const nox_msgs::SignalArray::ConstPtr &chassis_state)
{
    defines::Panel panel;
    panel.FromMsgs(*chassis_state);
    double throttle = panel.Throttle.Get();
    double brake = panel.Brake.Get();
    double speed = panel.Speed.Get();
    // ROS_INFO_STREAM("Panel Throttle is "<<throttle);
    // ROS_INFO_STREAM("Panel Brake is "<<brake);
    // ROS_INFO_STREAM("Panel Speed is "<<speed);
}
void Control::WriteInDebug(ofstream &ofs, prius_msgs::Augmented_My_Trajectory_Point vehicle_info,TrajectoryAnalyzer &trajectory_analyzer,ControllerAgent &controller_agent)
{
    // ofs<<"trajectory_point{"<<endl;
    // ofs<<"  time: "<<vehicle_info.vehicle_info.header.stamp.toSec()<<endl;
    // ofs<<"  x: "<<vehicle_info.trajectory_point.x<<endl;
    // ofs<<"  y: "<<vehicle_info.trajectory_point.y<<endl;
    // ofs<<"  velocity: "<<vehicle_info.trajectory_point.v<<endl;
    // ofs<<"  acceleration: "<<vehicle_info.trajectory_point.a<<endl;
    // ofs<<"  heading: "<<vehicle_info.trajectory_point.theta<<endl;
    // ofs<<"  relative_time: "<<vehicle_info.trajectory_point.relative_time<<endl;
    // ofs<<"  lateral_distance_error: "<<vehicle_info.lateral_distance_error<<endl;
    // ofs<<"  velocity_error: "<<vehicle_info.velocity_error<<endl;
    // ofs<<"  heading_error: "<<vehicle_info.heading_error<<endl;
    // ofs<<"  x_from_chassis: "<<vehicle_info.vehicle_info.localization.x<<endl;
    // ofs<<"  y_from_chassis: "<<vehicle_info.vehicle_info.localization.y<<endl;
    // ofs<<"  z_from_chassis: "<<vehicle_info.vehicle_info.localization.z<<endl;
    // ofs<<"  heading_from_chassis: "<<vehicle_info.vehicle_info.lateral_data.heading_angle<<endl;
    // ofs<<"  vel_from_localization: "<<vehicle_info.vehicle_info.longitudinal_data.vel_from_localization<<endl;
    // ofs<<"  vel_from_wheels: "<<vehicle_info.vehicle_info.longitudinal_data.vel_from_wheels<<endl;
    // ofs<<"  acc_from_wheels: "<<vehicle_info.vehicle_info.longitudinal_data.acceleration<<endl;
    // ofs<<"  travel_distance: "<<vehicle_info.vehicle_info.longitudinal_data.traveled_distance<<endl;
    // ofs<<"  steering_wheel_angle_actual: "<<vehicle_info.vehicle_info.lateral_data.steering_wheel_angle_actual<<endl;
    // ofs<<"  steering_wheel_expected: "<<vehicle_info.vehicle_info.lateral_data.steering_wheel_expected<<endl;
    // ofs<<"  steering_wheel_error: "<<vehicle_info.vehicle_info.lateral_data.steering_wheel_error<<endl;
    // ofs<<"  steering_wheel_cmd: "<<vehicle_info.vehicle_info.lateral_data.steering_wheel_cmd<<endl;
    // ofs<<"  fl_steering_angle_actual: "<<vehicle_info.vehicle_info.lateral_data.fl_steering_angle_actual<<endl;
    // ofs<<"  fr_steering_angle_actual: "<<vehicle_info.vehicle_info.lateral_data.fr_steering_angle_actual<<endl;
    // ofs<<"  single_track_steering_angle: "<<vehicle_info.vehicle_info.lateral_data.single_track_steering_angle<<endl;
    // ofs<<"  fl_steering_angle_expected: "<<vehicle_info.vehicle_info.lateral_data.fl_steering_angle_expected<<endl;
    // ofs<<"  fr_steering_angle_expected: "<<vehicle_info.vehicle_info.lateral_data.fr_steering_angle_expected<<endl;
    // ofs<<"  fl_steering_error: "<<vehicle_info.vehicle_info.lateral_data.fl_steering_error<<endl;
    // ofs<<"  fl_steering_cmd: "<<vehicle_info.vehicle_info.lateral_data.fl_steering_cmd<<endl;
    // ofs<<"  fr_steering_error: "<<vehicle_info.vehicle_info.lateral_data.fr_steering_error<<endl;
    // ofs<<"  fr_steering_cmd: "<<vehicle_info.vehicle_info.lateral_data.fr_steering_cmd<<endl;
    // ofs<<"  fl_wheel_angular_velocity: "<<vehicle_info.vehicle_info.longitudinal_data.fl_wheel_angular_velocity<<endl;
    // ofs<<"  fr_wheel_angular_velocity: "<<vehicle_info.vehicle_info.longitudinal_data.fr_wheel_angular_velocity<<endl;
    // ofs<<"  bl_wheel_angular_velocity: "<<vehicle_info.vehicle_info.longitudinal_data.bl_wheel_angular_velocity<<endl;
    // ofs<<"  br_wheel_angular_velocity: "<<vehicle_info.vehicle_info.longitudinal_data.br_wheel_angular_velocity<<endl;
    // ofs<<"  gas_percent: "<<vehicle_info.vehicle_info.longitudinal_data.gas_percent<<endl;
    // ofs<<"  fl_gas_torque: "<<vehicle_info.vehicle_info.longitudinal_data.fl_gas_torque<<endl;
    // ofs<<"  fr_gas_torque: "<<vehicle_info.vehicle_info.longitudinal_data.fr_gas_torque<<endl;
    // ofs<<"  bl_gas_torque: "<<vehicle_info.vehicle_info.longitudinal_data.bl_gas_torque<<endl;
    // ofs<<"  br_gas_torque: "<<vehicle_info.vehicle_info.longitudinal_data.br_gas_torque<<endl;
    // ofs<<"  fl_brake_Torque: "<<vehicle_info.vehicle_info.longitudinal_data.fl_brake_Torque<<endl;
    // ofs<<"  fr_brake_torque: "<<vehicle_info.vehicle_info.longitudinal_data.fr_brake_torque<<endl;
    // ofs<<"  bl_brake_torque: "<<vehicle_info.vehicle_info.longitudinal_data.bl_brake_torque<<endl;
    // ofs<<"  br_brake_torque: "<<vehicle_info.vehicle_info.longitudinal_data.br_brake_torque<<endl;
    // ofs<<"}"<<endl;   
    ofs<<"trajectory_point{"<<endl; 
    ofs<<"  x: "<<vehicle_info.trajectory_point.x<<endl;
    ofs<<"  y: "<<vehicle_info.trajectory_point.y<<endl;
    ofs<<"  velocity: "<<vehicle_info.trajectory_point.v<<endl;
    double theta=vehicle_info.trajectory_point.theta;
    if(abs(abs(theta)-PI)<0.05)
    {
        theta = PI;
    }
    ofs<<"  heading: "<<theta<<endl;
    ofs<<"  relative_time: "<<vehicle_info.trajectory_point.relative_time<<endl;
    ofs<<"  travel_distance: "<<vehicle_info.vehicle_info.longitudinal_data.traveled_distance<<endl;
    ofs<<"  slope: "<<vehicle_info.slope<<endl;
    ofs<<"  lateral_distance_error: "<<vehicle_info.lateral_distance_error<<endl;
    ofs<<"  velocity_error: "<<vehicle_info.velocity_error<<endl;
    ofs<<"  heading_error: "<<vehicle_info.heading_error<<endl;
    ofs<<"  computational_time: "<<vehicle_info.computational_time<<endl;
    ofs<<"  average_lateral_distance_error: "<<vehicle_info.average_lateral_distance_error<<endl;
    ofs<<"  average_velocity_error: "<<vehicle_info.average_velocity_error<<endl;
    ofs<<"  average_heading_error: "<<vehicle_info.average_heading_error<<endl;
    ofs<<"  average_computational_time: "<<vehicle_info.average_computational_time<<endl;
    ofs<<"  max_lateral_distance_error: "<<vehicle_info.max_lateral_distance_error<<endl;
    ofs<<"  max_velocity_error: "<<vehicle_info.max_velocity_error<<endl;
    ofs<<"  max_heading_error: "<<vehicle_info.max_heading_error<<endl;
    ofs<<"  max_computational_time: "<<vehicle_info.max_computational_time<<endl;
    ofs<<"}"<<endl; 
    ROS_INFO("Write in debug successfully!");
}


