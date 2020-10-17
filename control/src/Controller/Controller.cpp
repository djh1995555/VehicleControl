#include <control/Controller/Controller.h>


Status Controller::Reset()
{
    status.status = "OK";
    return status;
}
string Controller::GetName()
{
    return name;
} 
void Controller::Stop()
{
    ROS_INFO_STREAM(name<<"exited!");
    return;
}