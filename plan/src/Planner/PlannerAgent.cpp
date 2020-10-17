#include <planning/Planner/PlannerAgent.h>

Status PlannerAgent::Init(const PlanningConf &planning_conf)
{
    ROS_INFO("Planner agent is initialized!");
    Ts = planning_conf.planning_param.Ts;
    activate_planning_function = planning_conf.planning_param.Activate_planning_function;
    Status reg_status = RegisterPlanner(planning_conf);
    if(reg_status.status != "OK")
    {
        ROS_INFO("Planner registration failed!");
        status.status = "Failed";
        return status;       
    }
    ROS_INFO("Planner registration successfully!");
    CurrentPlanner();

    if(planner_name == "Lattice Planner")
    {
        Planner* lattice_planner_ptr = new LatticePlanner;
        planner_list.push_back(lattice_planner_ptr);
        ROS_INFO_STREAM(planner_name<<" is created successfully!");
    }


    for(vector<Planner*>::iterator it = planner_list.begin();it!=planner_list.end();it++)
    {
        (*it)->Init(planning_conf);
    }
    status.status = "OK";
    return status;
}
Status PlannerAgent::RegisterPlanner(const PlanningConf &planning_conf)
{
    planner_name = planning_conf.Planner_name;
    status.status = "OK";
    return status;
}
void PlannerAgent::CurrentPlanner()
{
    ROS_INFO_STREAM("Current planner is "<<planner_name);
}


Status PlannerAgent::Plan(VehicleState &vehicle_state, Frame &frame,prius_msgs::My_Trajectory &local_trajectory)
{
    for(vector<Planner*>::iterator it = planner_list.begin();it!=planner_list.end();it++)
    {
        (*it)->PlanOnReferenceLine(vehicle_state, frame,local_trajectory);
    }

    status.status = "OK";
    return status;  
}