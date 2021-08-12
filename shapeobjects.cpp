#include "shapeobjects.h"

ShapeObjects::ShapeObjects(cv::Point center, int radius)
{
    this->center = center;
    this->radius = radius;
}

cv::Point ShapeObjects::getCenter()
{
    return this->center;
}
void ShapeObjects::setCenter(cv::Point center)
{
    this->center = center;
}
int ShapeObjects::getRadius()
{
    return this->radius;
}
void ShapeObjects::setRadius(int radius)
{
    this->radius = radius;
}
