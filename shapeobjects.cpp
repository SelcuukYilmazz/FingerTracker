#include "shapeobjects.h"

ShapeObjects::ShapeObjects(cv::Point center, int radius,std::string objectType)
{
    this->center = center;
    this->radius = radius;
    this->lock = false;
    this->scan = true;
    this->objectType = objectType;
}
void ShapeObjects::startStartTime()
{
    this->start_time = time(nullptr);

}
time_t ShapeObjects::getStartTime()
{
   return this->start_time;
}
void ShapeObjects::startCurrentTime()
{
    this->current_time= time(nullptr);

}
time_t ShapeObjects::getCurrentTime()
{
    return this->current_time;
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
bool ShapeObjects::getLock()
{
    return this->lock;
}
void ShapeObjects::setLock(bool lock)
{
    this->lock = lock;
}
bool ShapeObjects::getScan()
{
    return this->scan;
}
void ShapeObjects::setScan(bool scan)
{
    this->scan = scan;
}
std::string ShapeObjects::getObjectType()
{
    return this->objectType;
}
void ShapeObjects::setPastFrame(cv::Point pastFrame)
{
    this->pastFrame = pastFrame;
}
void ShapeObjects::unlockShape(cv::Point updatedFrame)
{
    std::cout<<getScan()<<std::endl;
    if((abs(this->pastFrame.x-updatedFrame.x)<this->radius || abs(this->pastFrame.y-updatedFrame.y)<this->radius) && getScan())
    {
        setLock(false);
        setScan(!getScan());
    }
    else if(!getScan())
    {
        setScan(!getScan());
    }
}
