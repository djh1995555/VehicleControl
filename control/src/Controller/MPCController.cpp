#include<control/Controller/MPCController.h>

Status MPCController::Init(const ControlConf &control_conf)
{
    name = "MPC controller";
    prediction_length=control_conf.conf_param.MPC_predict_length;
    control_length=control_conf.conf_param.MPC_control_length;


    Q(0,0) = control_conf.conf_param.MPC_Q_0_0;
    Q(1,1) = control_conf.conf_param.MPC_Q_1_1;
    Q(2,2) = control_conf.conf_param.MPC_Q_2_2;
/*     R(0,0) = control_conf.conf_param.MPC_R_0_0;
    R(1,1) = control_conf.conf_param.MPC_R_1_1; */
    R(0,0) = control_conf.conf_param.MPC_R_1_1;
    ROS_INFO("load Q and R");

    Ts=control_conf.conf_param.Ts;
    wheel_base=control_conf.conf_param.wheel_base;
    Q_augmented=MatrixXd::Identity(prediction_length*Nx,prediction_length*Nx);
    R_augmented=MatrixXd::Identity(control_length*Nu,control_length*Nu);

    GetAugmentedMatrix(Q_augmented,Q,prediction_length);
    GetAugmentedMatrix(R_augmented,R,control_length);
    ROS_INFO_STREAM("Q(3,3) = "<<Q_augmented(3,3));
    ROS_INFO_STREAM("Q(4,4) = "<<Q_augmented(4,4));
    ROS_INFO_STREAM("Q(5,5) = "<<Q_augmented(5,5));
    ROS_INFO_STREAM("R(2,2) = "<<R_augmented(2,2));
    ROS_INFO_STREAM("R(3,3) = "<<R_augmented(3,3));
    ROS_INFO("load augmented Q and R");

    
    
    ROS_INFO_STREAM(name<<" is initialized successfully!");
    status.status = "OK";
    return status;

}
void MPCController::GetAugmentedMatrix(MatrixXd &matrix_augmented,const MatrixXd &matrix,int num)
{

    for(int i =0;i<num;i++)
    {
        matrix_augmented.block(i*matrix.rows(),i*matrix.cols(),matrix.rows(),matrix.cols())=matrix;
    }
}
void MPCController::CreatePredictionModel()
{
    vector<Matrix3d> A_power(prediction_length);
    
    A_power[0]=A;
    for(int i =1;i<A_power.size();++i)
    {
        A_power[i]=A*A_power[i-1];
    }
/*     ROS_INFO_STREAM("A_power[2](0,0) = "<<A_power[2](0,0));
    ROS_INFO_STREAM("A_power[2](0,1) = "<<A_power[2](0,1));
    ROS_INFO_STREAM("A_power[2](0,2) = "<<A_power[2](0,2));

    ROS_INFO_STREAM("A_power[2](1,0) = "<<A_power[2](1,0));
    ROS_INFO_STREAM("A_power[2](1,1) = "<<A_power[2](1,1));
    ROS_INFO_STREAM("A_power[2](1,2) = "<<A_power[2](1,2));

    ROS_INFO_STREAM("A_power[2](2,0) = "<<A_power[2](2,0));
    ROS_INFO_STREAM("A_power[2](2,1) = "<<A_power[2](2,1));
    ROS_INFO_STREAM("A_power[2](2,2) = "<<A_power[2](2,2)); */
    A_augmented = MatrixXd::Zero(prediction_length*A.rows(),A.cols());
    A_augmented.block(0,0,A.rows(),A.cols())=A;
    for(int i =1;i<prediction_length;++i)
    {
        A_augmented.block(i*A.rows(),0,A.rows(),A.cols())=A*A_augmented.block((i-1)*A.rows(),0,A.rows(),A.cols());
    }
/*     ROS_INFO_STREAM("A_augmented(6,0) = "<<A_augmented(6,0));
    ROS_INFO_STREAM("A_augmented(6,1) = "<<A_augmented(6,1));
    ROS_INFO_STREAM("A_augmented(6,2) = "<<A_augmented(6,2));

    ROS_INFO_STREAM("A_augmented(7,0) = "<<A_augmented(7,0));
    ROS_INFO_STREAM("A_augmented(7,1) = "<<A_augmented(7,1));
    ROS_INFO_STREAM("A_augmented(7,2) = "<<A_augmented(7,2));

    ROS_INFO_STREAM("A_augmented(8,0) = "<<A_augmented(8,0));
    ROS_INFO_STREAM("A_augmented(8,1) = "<<A_augmented(8,1));
    ROS_INFO_STREAM("A_augmented(8,2) = "<<A_augmented(8,2)); */

    B_augmented = MatrixXd::Zero(prediction_length*B.rows(),prediction_length*B.cols());
    B_augmented.block(0,0,B.rows(),B.cols())=B;

    for(int r=1;r<prediction_length;++r)
    {
        for(int c=0;c<r;c++)
        {
            B_augmented.block(r*B.rows(),c*B.cols(),B.rows(),B.cols())=A_power[r-c-1]*B;
        }
        B_augmented.block(r*B.rows(),r*B.cols(),B.rows(),B.cols())=B;
    }
/*     ROS_INFO_STREAM("B_augmented(3,0) = "<<B_augmented(3,0));
    ROS_INFO_STREAM("B_augmented(4,0) = "<<B_augmented(4,0));
    ROS_INFO_STREAM("B_augmented(5,0) = "<<B_augmented(5,0));
    ROS_INFO_STREAM("B_augmented(3,1) = "<<B_augmented(3,1));
    ROS_INFO_STREAM("B_augmented(4,1) = "<<B_augmented(4,1));
    ROS_INFO_STREAM("B_augmented(5,1) = "<<B_augmented(5,1)); */
}
void MPCController::ComputeStateError(const prius_msgs::My_Trajectory_Point &goal_state,const VehicleState &vehicle_state)
{
    double dx = vehicle_state.movement_state.pose.position.x-goal_state.x;
    double dy = vehicle_state.movement_state.pose.position.y-goal_state.y;
    double heading_error = vehicle_state.heading_angle-goal_state.theta;
    if(heading_error>pi)
    {
        heading_error=heading_error-2*pi;
    }
    else if(heading_error<-pi)
    {
        heading_error=heading_error+2*pi;
    }
    state_error(0)=dx;
    state_error(1)=dy;
    state_error(2)=heading_error;
    ROS_INFO_STREAM("the x error is "<<state_error(0));
    ROS_INFO_STREAM("the y error is "<<state_error(1));
    ROS_INFO_STREAM("the current heading is "<<vehicle_state.heading_angle);
    ROS_INFO_STREAM("the goal heading is "<<goal_state.theta);
    ROS_INFO_STREAM("the heading error is "<<state_error(2));

}   
double MPCController::GetPlannedSteeringAngle(const prius_msgs::My_Trajectory_Point point)
{
    double planned_steering_angle;
    if(point.kappa==0)
    {
        planned_steering_angle = 0;
    }
    else if(point.kappa>0)
    {
        planned_steering_angle=atan2(wheel_base,1/point.kappa);
    }
    else
    {
        planned_steering_angle=-atan2(wheel_base,1/(-point.kappa));
    }
    return planned_steering_angle;
}
Status MPCController::ComputeControlCmd(TrajectoryAnalyzer &trajectory_analyzer, const VehicleState &vehicle_state,prius_msgs::Control &control_cmd)
{
    ROS_INFO("Use MPC controller");
    ComputeStateError(trajectory_analyzer.goal_state,vehicle_state);
    current_velocity=vehicle_state.current_velocity;
    //planned_velocity=trajectory_analyzer.goal_state.v;
    planned_heading = trajectory_analyzer.goal_state.theta;
    planned_steering_angle=GetPlannedSteeringAngle(trajectory_analyzer.goal_state);

    A(0,2) = -current_velocity *sin(planned_heading)*Ts;
    A(1,2) = current_velocity*cos(planned_heading)*Ts;
/*     B(0,0) = Ts*cos(planned_heading);
    B(1,0) = Ts*sin(planned_heading);
    B(2,0) = Ts*tan(planned_steering_angle)/wheel_base;
    B(2,1) = planned_velocity*Ts/(wheel_base*cos(planned_steering_angle)*cos(planned_steering_angle)); */
    B(2,0) = current_velocity*Ts/(wheel_base*cos(planned_steering_angle)*cos(planned_steering_angle));



/*     ROS_INFO_STREAM("B(0,0) = "<<B(0,0));
    ROS_INFO_STREAM("B(1,0) = "<<B(1,0));
    ROS_INFO_STREAM("B(2,0) = "<<B(2,0));
    ROS_INFO_STREAM("B(0,1) = "<<B(0,1));
    ROS_INFO_STREAM("B(1,1) = "<<B(1,1));
    ROS_INFO_STREAM("B(2,1) = "<<B(2,1));

    ROS_INFO_STREAM("A(0,0) = "<<A(0,0));
    ROS_INFO_STREAM("A(0,1) = "<<A(0,1));
    ROS_INFO_STREAM("A(0,2) = "<<A(0,2));
    ROS_INFO_STREAM("A(1,0) = "<<A(1,0));
    ROS_INFO_STREAM("A(1,1) = "<<A(1,1));
    ROS_INFO_STREAM("A(1,2) = "<<A(1,2));
    ROS_INFO_STREAM("A(2,0) = "<<A(2,0));
    ROS_INFO_STREAM("A(2,1) = "<<A(2,1));
    ROS_INFO_STREAM("A(2,2) = "<<A(2,2));  */


    ROS_INFO_STREAM("update state");
    CreatePredictionModel();
    ROS_INFO_STREAM("Create Prediction Model!");

    MatrixXd H=2*(B_augmented.transpose()*Q_augmented*B_augmented+R_augmented);
    MatrixXd G=2*B_augmented.transpose()*Q_augmented*A_augmented*state_error;
    ROS_INFO_STREAM("Rows of H = "<<H.rows());
    ROS_INFO_STREAM("Cols of H = "<<H.cols());
    ROS_INFO_STREAM("Rows of G = "<<G.rows());
    ROS_INFO_STREAM("Cols of G = "<<G.cols());

    ROS_INFO_STREAM("Compute H and G!");

    double lb=-0.64;
    double ub=0.64;
/*     double lb=-0.64;
    double ub=0.64; */
    int dim = H.rows()*H.cols();
    int_t nV=H.rows();
    real_t h[dim];
    real_t g[nV];
    real_t LB[nV];
    real_t UB[nV];

    int index = 0;
    for(int r=0;r<H.rows();r++)
    {   
        g[r]=G(r,0);
        for(int c=0;c<H.cols();c++)
        {
            h[index++]=H(r,c);
        }
    }

    for(int i=0;i<nV;i++)
    {
        LB[i]=lb;
        UB[i]=ub;
    }
    ROS_INFO("Load condition");
    QProblemB QPSolver( nV );

/*     Options options;
	//options.enableFlippingBounds = BT_FALSE;
	options.initialStatusBounds = ST_INACTIVE;
	options.numRefinementSteps = 1;
	options.enableCholeskyRefactorisation = 1;
	QPSolver.setOptions( options ); */
 
    int_t nWSR = 25;

    returnValue result1=QPSolver.init( h,g,LB,UB, nWSR,0);
    ROS_INFO_STREAM("first "<<result1);

/*     returnValue result2 = QPSolver.hotstart( g,lb,ub, nWSR,0 );
    ROS_INFO_STREAM("second "<<result2); */

    /* Get and print solution of second QP. */
/*     real_t xOpt[2];
    QPSolver.getPrimalSolution( xOpt );
    ROS_INFO_STREAM("xOpt = [ "<<xOpt[0]<<" , "<<xOpt[1]<<" ]; objVal = "<<QPSolver.getObjVal()); */

    real_t xOpt[1];
    QPSolver.getPrimalSolution( xOpt );
    ROS_INFO_STREAM("xOpt = "<<xOpt[0]<<"; objVal = "<<QPSolver.getObjVal());

    double steering_angle = xOpt[1]+planned_steering_angle;
    previous_steering_angle = steering_angle;
    //double steering_angle = xOpt[1];

    control_cmd.steer=steering_angle;
    ROS_INFO("Compute lateral command successfully!");
    status.status = "OK";
    return status;
}
