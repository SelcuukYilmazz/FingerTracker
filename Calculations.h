#ifndef Calculations_H
#define Calculations_H

#include <opencv2/core/types.hpp>
#include <vector>
#define PI 3.14159265


class Calculations
{
public:
    Calculations();
    int findBiggestContour(std::vector<std::vector<cv::Point> > handContours);
    double calculate_angle(std::vector<cv::Point> points);
};

#endif // Calculations_H
