#include <iostream>
using namespace std;
#include <planning/Planning.h>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"Planning");
    Planning Planning;
    Planning.Spin();
    return 0;
}