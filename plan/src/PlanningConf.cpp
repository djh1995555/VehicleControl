#include<planning/PlanningConf.h>

Status PlanningConf::ReadControlConf(const string plan_conf_file_name)
{
    ifs.open(plan_conf_file_name, ios::in);
    if (!ifs.is_open())
    {
        ROS_INFO("read failed");
        status.status = "Failed";
        return status;
    }
    string s;
    getline(ifs,s);
    int index=s.find("=");
    Planner_name = s.substr(index+1,s.size()-index-1);
    ROS_INFO_STREAM("planner_name is "<<Planner_name);

    double *p = &(planning_param.Ts);
    while(getline(ifs,s))
    {
        index = s.find("=");
        
        double num;
        string data;
        if ( index != string::npos)
        {
            data = s.substr(index + 1, s.size()-index-1);
            istringstream iss(data);
            iss >> num;
            (*p++) = num;
        }
    }
    // printData();
    status.status = "OK";
    return status;
}

void PlanningConf::printData()
{
}