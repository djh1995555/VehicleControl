#include<planning/BoundingBox.h>

double BoundingBox::GetDiagonal()
{
    diagonal = sqrt(length*length+width*width);
    return diagonal;
}

void BoundingBox::SetHeading(double yaw)
{
    heading = yaw;
}