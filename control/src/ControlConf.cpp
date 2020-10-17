#include<control/ControlConf.h>

Status ControlConf::ReadControlConf(const string control_conf_file_name)
{
    ifs.open(control_conf_file_name, ios::in);
    if (!ifs.is_open())
    {
        cout << "读取失败" << endl;
        status.status = "Failed";
        return status;
    }
    string s;
    getline(ifs,s);
    int index=s.find("=");
    LONGITUDINAL_CONTROLLER = s.substr(index+1,s.size()-index-1);

    getline(ifs,s);
    index=s.find("=");
    LATERAL_CONTROLLER = s.substr(index+1,s.size()-index-1);

    getline(ifs,s);
    index=s.find("=");
    CENTRALIZED_CONTROLLER = s.substr(index+1,s.size()-index-1);
    
    ROS_INFO_STREAM("LONGITUDINAL_CONTROLLER is "<<LONGITUDINAL_CONTROLLER);
    ROS_INFO_STREAM("LATERAL_CONTROLLER is "<<LATERAL_CONTROLLER);
    ROS_INFO_STREAM("CENTRALIZED_CONTROLLER is "<<CENTRALIZED_CONTROLLER);


    double* ptr = &(conf_param.Ts);
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
            (*ptr++) = num;
        }
    }
    // printData();
    status.status = "OK";
    return status;
}

void ControlConf::printData()
{
    // ROS_INFO_STREAM("velocity_kp"<<conf_param.velocity_kp);
    // ROS_INFO_STREAM("velocity_ki"<<conf_param.velocity_ki);
    // ROS_INFO_STREAM("velocity_kd"<<conf_param.velocity_kd);
    // ROS_INFO_STREAM("brake_distance_coefficient"<<conf_param.brake_distance_coefficient);

}