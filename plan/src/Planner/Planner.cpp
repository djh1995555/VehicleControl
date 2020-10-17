#include <planning/Planner/Planner.h>


Status Planner::Reset()
{
    status.status = "OK";
    return status;
}
string Planner::GetName()
{
    return name;
} 
void Planner::Stop()
{
    ROS_INFO_STREAM(name<<"exited!");
    return;
}