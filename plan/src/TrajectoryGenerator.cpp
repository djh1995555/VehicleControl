#include <planning/TrajectoryGenerator.h>

void TrajectoryGenerator::GenerateTrajectory(const FrenetPathPoint &start_point,const FrenetPathPoint &end_point,polynomials &polys)
{
    ROS_INFO("GenerateTrajectory");
    GetPolynomial(start_point.s,start_point.s_d,start_point.s_dd,end_point.s,end_point.s_d,end_point.s_dd,end_point.t,polys.longitudinal_poly);
    for(int i =0;i<polys.longitudinal_poly.coefficient.size();++i)
    {
        ROS_INFO_STREAM("lon coef = "<<polys.longitudinal_poly.coefficient[i]);
    }
    GetPolynomial(start_point.d,start_point.d_d,start_point.d_dd,end_point.d,end_point.d_d,end_point.d_dd,end_point.s,polys.lateral_poly);
    for(int i =0;i<polys.lateral_poly.coefficient.size();++i)
    {
        ROS_INFO_STREAM("lat coef = "<<polys.lateral_poly.coefficient[i]);
    }
    CombineTrajectory(start_point,end_point,polys);

}

void TrajectoryGenerator::GetPolynomial(double start_x,double start_x_d,double start_x_dd,double end_x,double end_x_d,double end_x_dd,double param,Polynomial &poly)
{
    vector<double> coef;
    coef.push_back(start_x);
    coef.push_back(start_x_d);
    coef.push_back(start_x_dd / 2.0);
    double c1 = param*param;
    double c2 = c1*param;
    double c3 = c2*param;
    coef.push_back((10*(end_x-start_x)-(4*end_x_d+6*start_x_d)*param+(end_x_dd-3*start_x_dd)*c1/2)/c2);
    coef.push_back((15*(start_x-end_x)+(7*end_x_d+8*start_x_d)*param+(3*start_x_dd-2*end_x_dd)*c1/2)/c3);
    coef.push_back((6*(end_x-start_x)-3*(end_x_d+start_x_d)*param+(end_x_dd-start_x_dd)*c1/2)/(c1*c2));
    poly.Init(coef);
}

void TrajectoryGenerator::CombineTrajectory(const FrenetPathPoint &start_point,const FrenetPathPoint &end_point,polynomials &polys)
{
    ROS_INFO("CombineTrajectory");
    double end_s = end_point.s;
    double end_t = end_point.t;
    ROS_INFO_STREAM("end s is "<<end_s);
    ROS_INFO_STREAM("end t is "<<end_t);
    double t = 0;
    FrenetPathPoint point;
    while(t<end_t)
    {
        point.s = polys.longitudinal_poly.CalculateValue(t);
        point.s_d = polys.longitudinal_poly.CalculateFirstDerivative(t);
        point.s_dd = polys.longitudinal_poly.CalculatethirdDerivative(t);
        point.s_ddd = polys.longitudinal_poly.CalculatethirdDerivative(t);
        double s = point.s;
        point.d = polys.lateral_poly.CalculateValue(s);
        point.d_d = polys.lateral_poly.CalculateFirstDerivative(s);
        point.d_dd = polys.lateral_poly.CalculatesecondDerivative(s);
        point.d_ddd = polys.lateral_poly.CalculateValue(s);
        point.t = t;

        polys.frenet_path.FrenectPoints.push_back(point);
        t += time_interval;
    }
    ROS_INFO_STREAM("size of frenet path is "<<polys.frenet_path.FrenectPoints.size());
}

double TrajectoryGenerator::ComputeCost(FrenetPath &frenet_path,const double target_velocity)
{
    ROS_INFO("ComputeCost");
    FrenetPathPoint end_point = frenet_path.FrenectPoints.back();
    ROS_INFO_STREAM("end_point.s_d is "<<end_point.s_d);
    double velocity_error = pow(target_velocity - end_point.s_d,2);
    double longitudinal_jerk_sum = 0;
    double lateral_jerk_sum = 0;
    double lateral_derivation = 0;
    for(int i =0;i<frenet_path.FrenectPoints.size();++i)
    {
        if(i%50==0)
        {
            ROS_INFO_STREAM("get lon jerk of "<<i<<" point is "<<frenet_path.FrenectPoints[i].s_ddd);
            ROS_INFO_STREAM("get lat jerk of "<<i<<" point is "<<frenet_path.FrenectPoints[i].d_ddd);
        }
        if(frenet_path.FrenectPoints[i].s>frenet_path.FrenectPoints.back().s)
        {
            frenet_path.is_valid = false;
            
            return -1;
        }
        longitudinal_jerk_sum += pow(frenet_path.FrenectPoints[i].s_ddd,2);
        lateral_jerk_sum += pow(frenet_path.FrenectPoints[i].d_ddd,2);
        lateral_derivation += pow(frenet_path.FrenectPoints[i].d,2);
    }
    double longitudinal_cost = K_J * longitudinal_jerk_sum+K_T*end_point.t+K_D*velocity_error;
    double lateral_cost = K_J * lateral_jerk_sum+K_T*end_point.t+K_D*lateral_derivation;
    ROS_INFO_STREAM("COST = "<<K_lon * longitudinal_cost + K_lat*lateral_cost);
    return K_lon * longitudinal_cost + K_lat*lateral_cost; 
}

