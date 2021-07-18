#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <ctime>
#include <math.h>
#include <iostream>
#define PI 3.14159265
using namespace cv;
using namespace std;

Mat frame,output,circleFrame;
Mat imgBlur, imgBilateral, imgCanny,imgDilate,imgGray;
Mat output_approx, imgTransparent, circleGray;
vector<Point2f> boxPts(4);
String objectType;
const int fps =30;
double alpha = 0.6;
double beta;
time_t current_time,start_time,past_time;
vector<Point> tempPoly;
vector<vector<Point>> conPoly;
vector<Rect> boundRect;


double angle(vector<Point> points)
{
    double sumOfAngles = 0;
    double x_distance, y_distance;
    for(int i=0;i<points.size();i++)
    {
        if (i !=points.size()-2)
        {
            x_distance = abs(points[i+2].x-points[i].x);
            y_distance = abs(points[i+2].y-points[i].y);
            sumOfAngles = sumOfAngles + atan (x_distance / y_distance) * 180 / PI;
        }
        else{
            x_distance = abs(points[i-2].x-points[i].x);
            y_distance = abs(points[i-2].y-points[i].y);
            sumOfAngles = sumOfAngles + atan (x_distance / y_distance) * 180 / PI;
        }
    }
    return sumOfAngles;
}


void getContours(Mat output_canny,Mat output,Mat circleFrame)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

//  findContours function finding contours with that code can detect shapes
    findContours( output_canny, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );


//    with detected contours we can detect all shapes

    current_time = time(nullptr);
    past_time = current_time-start_time;
    if(past_time>3){
        start_time = time(nullptr);

        conPoly.clear();
        tempPoly.clear();
        boundRect.clear();
        for(int i = 0; i < contours.size();i++)
        {

            vector<RotatedRect> minRect( contours.size() );

    //        This functions calculates area of detected shape and if area is bigger than 1000 pixels it passes
            int area = contourArea(contours[i]);

                if (area > 1000)
                {
    //            This code finding shape's corners
                    float peri = arcLength(contours[i],true);
                    approxPolyDP(contours[i],tempPoly,0.03*peri,true);
                    conPoly.push_back(tempPoly);

                }
        }
    }



            for(int i = 0; i < conPoly.size();i++){
                int objCor = (int)conPoly[i].size();
                boundRect.push_back(boundingRect(conPoly[i]));
                rectangle(output,boundRect[i].tl(),boundRect[i].br(),Scalar(0,255,0),5);
        //                  Determining shape names below
                if (objCor == 3){
        //                If corner amount is lesser or equal to 6 then this code fills shape
                    drawContours(output,conPoly,i,Scalar(0,0,0),FILLED);
                    objectType = "Ucgen";
                    if(past_time>3){
                        cout<<angle(conPoly[i])<<endl<<"Ucgen"<<endl;
                    }
                }
                else if (objCor == 4){
        //                This code determines if the shape is rectangle or square by looking edges
                    double ratio = (double)(boundRect[i].width / boundRect[i].height);
                    if (ratio >=0.90 and ratio <= 1.10)
                    {
        //                If corner amount is lesser or equal to 6 then this code fills shape
                        drawContours(output,conPoly,i,Scalar(100,0,0),FILLED);
                        objectType = "Kare";
                        if(past_time>3){
                            cout<<angle(conPoly[i])<<endl<<"Kare"<<endl;
                        }
                    }
                    else
                    {
        //                If corner amount is lesser or equal to 6 then this code fills shape
                        drawContours(output,conPoly,i,Scalar(200,0,200),FILLED);
                        objectType = "Dortgen";
                        if(past_time>3){
                            cout<<angle(conPoly[i])<<endl<<"Dortgen"<<endl;
                        }
                    }
                }
                else if (objCor == 5){
        //                If corner amount is lesser or equal to 6 then this code fills shape
                    drawContours(output,conPoly,i,Scalar(0,100,0),FILLED);
                    objectType = "Besgen";
                    if(past_time>3){
                        cout<<angle(conPoly[i])<<endl<<"Besgen"<<endl;
                    }
                }
                else if (objCor == 6){
        //                If corner amount is lesser or equal to 6 then this code fills shape
                    drawContours(output,conPoly,i,Scalar(0,200,200),FILLED);
                    objectType = "Altigen";
                    if(past_time>3){
                        cout<<angle(conPoly[i])<<endl<<"Altigen"<<endl;
                    }
                }
                else if (objCor <=8){
        //                If shape has more than 6 corners then finding code uses HoughCircles method
        //                cvtColor(circleFrame, circleGray, COLOR_BGR2GRAY);
        //                medianBlur(circleGray, circleGray, 5);
        //                vector<Vec3f> circles;
        //                HoughCircles(circleGray, circles, HOUGH_GRADIENT, 1,
        //                                 output.rows/64,  // change this value to detect circles with different distances to each other
        //                                 100, 40, 200, 500 // change the last two parameters
        //                            // (min_radius & max_radius) to detect larger circles
        //                    );
        //                for( size_t i = 0; i < circles.size(); i++ )
        //                    {
        //                        Vec3i c = circles[i];
        //                        Point center = Point(c[0], c[1]);
        //                        // circle outline
        //                        int radius = c[2];
        //                        circle( output, center, radius, Scalar(0,0,255),FILLED,LINE_8);
        //                    }
                    drawContours(output,conPoly,i,Scalar(0,0,255),FILLED);
                    objectType = "Daire";
                    if(past_time>3){
                        cout<<angle(conPoly[i])<<endl<<"Daire"<<endl;
                    }
                }
        //            Puts texts of shapes onto them
                putText(frame,objectType,{boundRect[i].x,boundRect[i].y - 5},FONT_HERSHEY_PLAIN,3,Scalar(0,69,255),2);
            }

}

int main()
{
    start_time = time(nullptr);
//    capture variable is for direct photo inputs
    string capture = samples::findFile("/home/selcuk/SIMTEK/daire.png");
//    VideoCapture class is for capturing frames from webcams
    VideoCapture vid(0);
    vid.set(CAP_PROP_FRAME_WIDTH,640);
    vid.set(CAP_PROP_FRAME_HEIGHT,480);

//    This if statement is checking if the code took frame from webcam or not if not code quitting.
    if (!vid.isOpened())
    {
        return -1;
    }
//     This while loop is reading each frame from camera and using it
    while(vid.read(frame))
    {
//        output is clone of frame so we can do some changes on output and tranparenting it on frame
        output = frame.clone();
        circleFrame = frame.clone();

    //    Preprocessing Image
//        Converting image BGR color space to Grayscale color space
        cvtColor(frame,imgGray,COLOR_BGR2GRAY);
//        Blurring image so we can detect shapes better. But not overdoing it because in that case code won't detect anything
        GaussianBlur(imgGray,imgBlur,Size(3,3),3,0);
//        Canny filter is detecting edges better
        Canny( imgBlur, imgCanny, 25, 75);
//        Creating kernel for using it in dilate function. with Dilate we can detect better and easily.
        Mat kernel = getStructuringElement(MORPH_RECT,Size(5,5));
        dilate(imgCanny,imgDilate,kernel);
//        Calling getContours function
            getContours(imgDilate,output,circleFrame);

//        Alpha and Beta value is for tranparenting parameters
        beta = (1.0-alpha);
//        addWeighted function is transparenting image
        addWeighted(frame,alpha,output,beta,0.0,imgTransparent);
//        Imshow showing image


        imshow("Trans",imgTransparent);
//        This if statement setting fps
        if (waitKey(1000/fps)>=0){
            VideoCapture release(0);
            break;
            }

    }

    return 0;
}
