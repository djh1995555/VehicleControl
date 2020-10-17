#include <iostream>
using namespace std;
#include <control/Control.h>

int main(int argc, char **argv)
{
    
    ros::init(argc,argv,"ControlMudule");
    Control Control;
    Control.Spin();
    return 0;
}