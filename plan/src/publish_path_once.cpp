#include<iostream>
using namespace std;
#include<ros/ros.h>
#include<fstream>
#include<vector>
#include<prius_msgs/Control.h>
#include<prius_msgs/My_Trajectory.h>

const double PI=3.14159236;
const int VELOCITY_FACTOR=2;


void FindAll(string s, char ch,vector<int> &pos)
{
    for(int i=0;i<s.length();i++)
    {
        if(s[i]==ch)
        {
            pos.push_back(i);
        }
    }
}
void GetData(int start,int end,string s,double &target)
{
    string data = s.substr(start,end-start);
    istringstream iss(data);
    iss >> target;   
}

// read waytous simulation trajectory
int ReadTrueTrajectory(prius_msgs::My_Trajectory &my_trajectory, ifstream& ifs)
{
    string s;
    int i=0;
    while(getline(ifs,s))
    {
        
        prius_msgs::My_Trajectory_Point point;
        vector<int> pos;
        pos.push_back(-1);
        FindAll(s,'\t',pos);
        ROS_INFO("before");
        GetData(pos[0]+1,pos[1],s,point.x);
        GetData(pos[1]+1,pos[2],s,point.y);
        GetData(pos[3]+1,pos[4],s,point.theta);
        ROS_INFO("after");
        point.theta=point.theta*PI/180;
        GetData(pos[5]+1,pos[6],s,point.v);
        point.v=point.v*VELOCITY_FACTOR;
        my_trajectory.trajectory_points.push_back(point);
        ROS_INFO_STREAM("read "<<i++<<" point");
    }
    return 0;   
}

/* float GetNum(string s)
{
    int index = s.find(":");
    int size;
    size = s.size();
    string ss;
    ss = s.substr(index + 2, size - 1);
    istringstream iss(ss);
    float num;
    iss >> num;
    return num;
} */


// read trajectory file generated from matlab
int ReadFromFile(prius_msgs::My_Trajectory &my_trajectory, ifstream& ifs)
{

    string s;
    while(getline(ifs,s))
    {
        prius_msgs::My_Trajectory_Point point;
        double* p = &(point.x);
        while (true)
        {
            getline(ifs,s);
            int index = s.find(":");
            int size;
            double num;
            string data;
            if ( index != string::npos)
            {
                size = s.size();
                data = s.substr(index + 2, size-index-2);
                istringstream iss(data);
                iss >> num;
                (*p++) = num;
            }
            else if(s[0]=='}')
            {
                break;
            }
        }
        my_trajectory.trajectory_points.push_back(point);

    }
    return 0;
}

void PrintData(prius_msgs::My_Trajectory &my_trajectory, int index)
{
    ROS_INFO_STREAM("stamp"<<my_trajectory.header.stamp);
    ROS_INFO_STREAM("seq"<<my_trajectory.header.seq);
    prius_msgs::My_Trajectory_Point point = my_trajectory.trajectory_points[index];
    ROS_INFO_STREAM("x="<< point.x);
    ROS_INFO_STREAM("y=" << point.y);
    ROS_INFO_STREAM("z=" << point.z);
    ROS_INFO_STREAM("theta=" << point.theta);
    ROS_INFO_STREAM("kappa=" << point.kappa);
    ROS_INFO_STREAM("s=" << point.s);
    ROS_INFO_STREAM("dkappa=" << point.dkappa);
    ROS_INFO_STREAM( "v=" << point.v);
    ROS_INFO_STREAM("a=" << point.a);
    ROS_INFO_STREAM("relative_time=" << point.relative_time);
    ROS_INFO_STREAM("gear=" << point.gear);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_publisher");
    ros::NodeHandle node;
    ros::Publisher trajectory_pub=node.advertise<prius_msgs::My_Trajectory>("/global_trajectory",10);

    string file_path;
    node.getParam("test_trajectory_path", file_path);

    //string file_path="/home/djh/ControlModule/src/plan/test_trajectories/mat_trajectory_10.txt";

    int index=file_path.find(".");
    int index2=file_path.rfind("_",index);
    string trajectory_velocity;
    node.param<string>("trajectory_velocity", trajectory_velocity, "xx");
    trajectory_velocity=file_path.substr(index2+1,index-index2-1);
    node.setParam("trajectory_velocity",trajectory_velocity);
    ROS_INFO_STREAM("velocity is "<<trajectory_velocity);


    int pos = file_path.rfind("/",index);
    int pos2= file_path.find("_",pos);
    string trajectory_type=file_path.substr(pos+1,pos2-pos-1);
    ROS_INFO_STREAM("trajectory_type is "<<trajectory_type);



    ifstream ifs;
    ifs.open(file_path, ios::in);
    if (!ifs.is_open())
    {
        ROS_INFO("读取失败");
        return 0;
    }

    prius_msgs::My_Trajectory my_trajectory;
    if(trajectory_type=="mat"||trajectory_type=="recorded")
    {
        ReadFromFile(my_trajectory,ifs);
    }
    else if(trajectory_type=="true")
    {
        ReadTrueTrajectory(my_trajectory,ifs);
    }
    
    


    sleep(3);
    trajectory_pub.publish(my_trajectory);


    int i = 0;
    while(i<1401)
    {
        ROS_INFO_STREAM("the "<<i<<" trajectory");
        //PrintData(my_trajectory,i);
        i=i+100;
    }

    return 0;
}
