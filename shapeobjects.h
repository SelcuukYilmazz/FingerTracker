#ifndef SHAPEOBJECTS_H
#define SHAPEOBJECTS_H

#include <opencv2/core/types.hpp>
#include <vector>

class ShapeObjects
{
private:
    cv::Point center;
    int radius;
public:
    ShapeObjects(cv::Point center,int radius);
    cv::Point getCenter();
    void setCenter(cv::Point center);
    int getRadius();
    void setRadius(int radius);
};

#endif // SHAPEOBJECTS_H
