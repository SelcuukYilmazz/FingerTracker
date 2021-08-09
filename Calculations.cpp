#include "Calculations.h"

Calculations::Calculations()
{
}


//  This method returns biggest size contour.
int Calculations::findBiggestContour(std::vector<std::vector<cv::Point> > handContours){
    int indexOfBiggestContour = -1;
    int sizeOfBiggestContour = 0;
    for (int i = 0; i < handContours.size(); i++){
        if(handContours[i].size() > sizeOfBiggestContour && handContours[i].size()>250){
            sizeOfBiggestContour = handContours[i].size();
            indexOfBiggestContour = i;
        }
    }
    return indexOfBiggestContour;
}


//  This method calculates angles between points
double Calculations::calculate_angle(std::vector<cv::Point> points)
{
    double sumOfAngles = 0;
    double x_distance, y_distance, hypotenuse1, hypotenuse2, hypotenuse3;
    for(int i=0;i<points.size();i++)
    {
//        We take points that has 1 different point between them because 2 point's angle must be 0.
        if (i !=points.size()-2)
        {
//            Code calculate distance between 3 points then calculates angle by triangle formula (acos((a^2+b^2-c^2)/(2*a*b)))
            x_distance = abs(points[i+1].x-points[i].x);
            y_distance = abs(points[i+1].y-points[i].y);
            hypotenuse1 = sqrt(pow(x_distance,2)+pow(y_distance,2));
            x_distance = abs(points[i+2].x-points[i+1].x);
            y_distance = abs(points[i+2].y-points[i+1].y);
            hypotenuse2 = sqrt(pow(x_distance,2)+pow(y_distance,2));
            x_distance = abs(points[i+2].x-points[i].x);
            y_distance = abs(points[i+2].y-points[i].y);
            hypotenuse3 = sqrt(pow(x_distance,2)+pow(y_distance,2));
            sumOfAngles = sumOfAngles + (acos((pow(hypotenuse1,2) + pow(hypotenuse2,2) - pow(hypotenuse3,2))/(2.0 * hypotenuse1 * hypotenuse2))* 180 / PI);
        }
//        This else statement can find 2 different points after vector size.
        else
        {
//            Code calculate distance between 3 points then calculates angle by triangle formula (acos((a^2+b^2-c^2)/(2*a*b)))
            x_distance = abs(points[i-1].x-points[i].x);
            y_distance = abs(points[i-1].y-points[i].y);
            hypotenuse1 = sqrt(pow(x_distance,2)+pow(y_distance,2));
            x_distance = abs(points[i-2].x-points[i-1].x);
            y_distance = abs(points[i-2].y-points[i-1].y);
            hypotenuse2 = sqrt(pow(x_distance,2)+pow(y_distance,2));
            x_distance = abs(points[i-2].x-points[i].x);
            y_distance = abs(points[i-2].y-points[i].y);
            hypotenuse3 = sqrt(pow(x_distance,2)+pow(y_distance,2));
            sumOfAngles = sumOfAngles + (acos((pow(hypotenuse1,2) + pow(hypotenuse2,2) - pow(hypotenuse3,2))/(2.0 * hypotenuse1 * hypotenuse2))* 180 / PI);
        }
    }
    return sumOfAngles;
}
