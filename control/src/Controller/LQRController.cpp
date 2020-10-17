#include <control/Controller/LQRController.h>

Status LQRController::Init(const ControlConf &control_conf)
{
    name = "LQR controller";
    Ts=control_conf.conf_param.Ts;
    wheel_base=control_conf.conf_param.wheel_base;
    max_steer=control_conf.conf_param.max_steer;
    LQR_preview_length = control_conf.conf_param.LQR_preview_length;
    LQR_prediction_iteration_num=control_conf.conf_param.LQR_prediction_iteration_num;
    solve_tolerance=control_conf.conf_param.solve_tolerance;
    max_iteration_num=control_conf.conf_param.max_iteration_num;
    prediction_enabled=control_conf.conf_param.prediction_enabled;

    // R(0,0)=control_conf.conf_param.LQR_R;
    R(0,0)=control_conf.conf_param.LQR_R;
    Q(0,0)=control_conf.conf_param.LQR_Q_0_0;
    Q(1,1)=control_conf.conf_param.LQR_Q_1_1;
    Q(2,2)=control_conf.conf_param.LQR_Q_2_2;

    preview_feedforward_coefficient = control_conf.conf_param.preview_feedforward_coefficient;
    cout<<name<<" initialized successfully!"<<endl;
    status.status = "OK";
    return status;

}
void LQRController::ComputeStateError(const prius_msgs::My_Trajectory_Point &goal_state,const VehicleState &vehicle_state)
{
    double dx=vehicle_state.movement_state.pose.position.x-goal_state.x;
    double dy=vehicle_state.movement_state.pose.position.y-goal_state.y;
    double dy_new=dy*cos(goal_state.theta)-dx*sin(goal_state.theta);
    state_error(0)=dx;
    state_error(1)=dy;
    double d_theta =vehicle_state.heading_angle-goal_state.theta;
    if(d_theta>pi)
    {
        d_theta=d_theta-2*pi;
    }
    else if(d_theta<-pi)
    {
        d_theta=d_theta+2*pi;
    }
    state_error(2) = d_theta;
/*     ROS_INFO_STREAM("the x error is "<<state_error(0));
    ROS_INFO_STREAM("the y error is "<<state_error(1));
    ROS_INFO_STREAM("the current heading is "<<vehicle_state.heading_angle);
    ROS_INFO_STREAM("the goal heading is "<<goal_state.theta);
    ROS_INFO_STREAM("the heading error is "<<state_error(2)); */
}
double LQRController::GetPreviewSteeringAngle(const prius_msgs::My_Trajectory_Point point)
{
    double preview_steering_angle;
    if(point.kappa==0)
    {
        preview_steering_angle = 0;
    }
    else if(point.kappa>0)
    {
        preview_steering_angle=atan2(wheel_base,1/point.kappa);
    }
    else
    {
        preview_steering_angle=-atan2(wheel_base,1/(-point.kappa));
    }
    return preview_steering_angle;
}
void LQRController::LQRSolverWithPrediction(TrajectoryAnalyzer &trajectory_analyzer,const VehicleState &vehicle_state)
{
    int num=1;
    int iteration_interval = floor(LQR_preview_length/LQR_prediction_iteration_num);
    for(int i= LQR_preview_trajectory.size()-1;i>=0;i=i-iteration_interval)
    {
        //ROS_INFO_STREAM("the "<<num++<<" iteration!");
        double steering_angle_in_trajectory=GetPreviewSteeringAngle(LQR_preview_trajectory[i]);
        UpdateKineticModel(LQR_preview_trajectory[i].v,LQR_preview_trajectory[i].theta,steering_angle_in_trajectory);
        Pk_new = Pk;
        Vk_new = Vk;

        K=(B.transpose()*Pk_new*B+R).inverse()*(B.transpose()*Pk_new*A);
        Ku=(B.transpose()*Pk_new*B+R).inverse()*R ;
        Kv=(B.transpose()*Pk_new*B+R).inverse()*B.transpose();

        Pk=A.transpose()*Pk_new*(A-B*K)+Q;
        
        Vk=(A-B*K).transpose()*Vk_new-K.transpose()*R*steering_angle_in_trajectory;
    }
    feedback = -K*state_error-Ku*feedforward(0)-Kv*Vk_new;
     
/*     ROS_INFO_STREAM("Ku is "<<Ku);
    ROS_INFO_STREAM("Kv for x error is "<<Kv(0));
    ROS_INFO_STREAM("Kv for y error is "<<Kv(1));
    ROS_INFO_STREAM("Kv for heading error is "<<Kv(2));  */
}



void LQRController::UpdateKineticModel(double planned_velocity,double planned_heading,double previous_steering_angle)
{
    A(0,2) = -planned_velocity*sin(planned_heading)*Ts;
    A(1,2) = planned_velocity*cos(planned_heading)*Ts;
    B(2) = planned_velocity*Ts/(wheel_base*cos(previous_steering_angle)*cos(previous_steering_angle));
}
void LQRController::SolveLQRProblem(const double tolerance,const int max_iteration_num)
{
    MatrixXd P = Q;
    int num_of_iteration = 0;
    double diff=numeric_limits<double>::max();
    MatrixXd P_last ;
    while(num_of_iteration<max_iteration_num&&diff>tolerance)
    {
        P_last = A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A+Q;
        diff =fabs((P_last-P).maxCoeff());
        P=P_last;
        num_of_iteration++;
    } 
    if(num_of_iteration>=max_iteration_num)
    {
        ROS_INFO_STREAM("LQR Solver cannot converge to a solution. The last diff is "<<diff);
    }
    else
    {
        ROS_INFO_STREAM("LQR SOlver converged at iteration:"<<num_of_iteration<<", final diff is "<<diff);
    }
    K = (R+B.transpose()*P*B).inverse()*B.transpose()*P*A;

    feedback = -K*state_error;
}
void LQRController::LQRSOlver(const prius_msgs::My_Trajectory_Point &goal_state)
{
    
    UpdateKineticModel(goal_state.v,goal_state.theta,previous_steering_angle);
    SolveLQRProblem(solve_tolerance, max_iteration_num);
}

Status LQRController::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer,const VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    ROS_INFO("Use LQR controller");
    current_velocity=vehicle_state.current_velocity;
    ComputeStateError(trajectory_analyzer.goal_state,vehicle_state);
    
    LQR_preview_trajectory=trajectory_analyzer.GetPreviewTrajectory(LQR_preview_length);
    int index = preview_feedforward_coefficient*LQR_preview_length;
    double preview_steering_angle = GetPreviewSteeringAngle(LQR_preview_trajectory[index]);
    feedforward(0) = preview_steering_angle;

    if(prediction_enabled==0)   
    {
        LQRSOlver(trajectory_analyzer.goal_state);
        ROS_INFO("use normal LQR");
    }
    else
    {
        LQRSolverWithPrediction(trajectory_analyzer, vehicle_state);
        ROS_INFO("use LQR with prediction");
    }
    steering_angle = feedback(0)+feedforward(0);
    previous_steering_angle = steering_angle;
    control_cmd.steer=steering_angle;
    ROS_INFO_STREAM("steering angle="<<steering_angle);
/*     ROS_INFO_STREAM("feedforward is "<<feedforward(0));  
    ROS_INFO_STREAM("feedback is "<<feedback(0)); 
    ROS_INFO_STREAM("A(0,2) is "<<A(0,2));
    ROS_INFO_STREAM("A(1,2) is "<<A(1,2));
    ROS_INFO_STREAM("B() is "<<B(2));
    ROS_INFO_STREAM("K for x error is "<<K(0));
    ROS_INFO_STREAM("K for y error is "<<K(1));
    ROS_INFO_STREAM("K for heading error is "<<K(2)); */

    ROS_INFO("Compute lateral command successfully!");
    status.status = "OK";
    return status;
}