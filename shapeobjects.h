#ifndef SHAPEOBJECTS_H
#define SHAPEOBJECTS_H

#include <opencv2/core/types.hpp>
#include <vector>
#include <ctime>
#include <iostream>

class ShapeObjects
{
private:
    cv::Point center,pastFrame;
    int radius,blue,red,green;
    std::string objectType;
    bool lock,scan;
    time_t current_time,start_time;
public:
    ShapeObjects(cv::Point center,int radius,std::string objectType,int blue, int red, int green);
    cv::Point getCenter();
    void startStartTime();
    void startCurrentTime();
    time_t getStartTime();
    time_t getCurrentTime();
    void setCenter(cv::Point center);
    int getRadius();
    void setRadius(int radius);
    bool getLock();
    void setLock(bool lock);
    bool getScan();
    void setScan(bool scan);
    void setPastFrame(cv::Point pastFrame);
    void unlockShape(cv::Point updatedFrame);
    std::string getObjectType();
    int getBlue();
    int getRed();
    int getGreen();
};

#endif // SHAPEOBJECTS_H
