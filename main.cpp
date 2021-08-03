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

Mat frame,output,circleFrame,frame_mask_hsv,frame_mask_YCbCr,frame_mask_lab;
Mat imgBlur, imgBilateral, imgCanny,imgDilate,imgGray;
Mat output_approx, imgTransparent, circleGray;
vector<Point2f> boxPts(4);
String objectType;
const int fps =30;
double alpha = 0.6;
double beta;
vector<double> shapeAngles;
time_t current_time,start_time,past_time;
vector<Point> tempPoly;
vector<vector<Point>> conPoly;
vector<Rect> boundRect;
vector<Vec3f> circles;
vector<int> shape_areas;
Mat hsvchannel[3],ycrcbchannel[3];

// Mouse click function
void mouseEvent( int evt, int x, int y, int d, void *param )
{
    Mat* values = (Mat*) param;
    if (evt == EVENT_LBUTTONDOWN)
    {
        cout<<"values"<<(int)(*values).at<Vec3b>(y, x)[0]<<" "<<(int)(*values).at<Vec3b>(y, x)[1]<<" "<<(int)(*values).at<Vec3b>(y, x)[2]<<endl;
    }
}

//  Find biggest contour method
int findBiggestContour(vector<vector<Point> > handContours){
    int indexOfBiggestContour = -1;
    int sizeOfBiggestContour = 0;
    for (int i = 0; i < handContours.size(); i++){
        if(handContours[i].size() > sizeOfBiggestContour){
            sizeOfBiggestContour = handContours[i].size();
            indexOfBiggestContour = i;
        }
    }
    return indexOfBiggestContour;
}


//  This method calculates angles between points
double calculate_angle(vector<Point> points)
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


void getContours(Mat output_canny,Mat output,Mat circleFrame)
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

//  findContours function finding contours with that code can detect shapes
    findContours( output_canny, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );


//    with detected contours we can detect all shapes
//    This line gets current time while executing code
    current_time = time(nullptr);
//    This line gets past time with subtraction of current time and start time
    past_time = current_time-start_time;
//    If past time is more than 3 seconds then it passes this if statement and this statement resets all vectors every 3 seconds and recalculates them.
    if(past_time>1)
    {
        start_time = time(nullptr);

        conPoly.clear();
        tempPoly.clear();
        boundRect.clear();
        shapeAngles.clear();
        circles.clear();
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
                    shapeAngles.push_back(calculate_angle(tempPoly));
                    shape_areas.push_back(area);
                }
        }
    }



            for(int i = 0; i < conPoly.size();i++)
            {
                drawContours(output,conPoly,i,Scalar(255,0,255),5);
                int objCor = (int)conPoly[i].size();
                boundRect.push_back(boundingRect(conPoly[i]));
//                  Determining shape names below
                if (objCor == 3)
                {
//                If corner amount is lesser or equal to 6 then this code fills shape

                    if(shapeAngles[i]>=80 && shapeAngles[i]<=280)
                    {
                        rectangle(output,boundRect[i].tl(),boundRect[i].br(),Scalar(0,255,0),5);
                        drawContours(output,conPoly,i,Scalar(0,0,0),FILLED);
                        objectType = "Ucgen";
                        putText(frame,objectType,{boundRect[i].x,boundRect[i].y - 5},FONT_HERSHEY_PLAIN,3,Scalar(0,69,255),2);
                    }

                }
                else if (objCor == 4)
                {
//                This code determines if the shape is rectangle or square by looking edges
                    double ratio = (double)(boundRect[i].width / boundRect[i].height);

                    if(shapeAngles[i]>=260 && shapeAngles[i]<=460)
                    {
                        rectangle(output,boundRect[i].tl(),boundRect[i].br(),Scalar(0,255,0),5);
                        if (ratio >=0.90 and ratio <= 1.10)
                        {
//                If corner amount is lesser or equal to 6 then this code fills shape
                            drawContours(output,conPoly,i,Scalar(100,0,0),FILLED);
                            objectType = "Kare";
                            putText(frame,objectType,{boundRect[i].x,boundRect[i].y - 5},FONT_HERSHEY_PLAIN,3,Scalar(0,69,255),2);
                        }
                        else
                        {
//                If corner amount is lesser or equal to 6 then this code fills shape
                            drawContours(output,conPoly,i,Scalar(200,0,200),FILLED);
                            objectType = "Dortgen";
                            putText(frame,objectType,{boundRect[i].x,boundRect[i].y - 5},FONT_HERSHEY_PLAIN,3,Scalar(0,69,255),2);
                        }
                    }

                }
                else if (objCor == 5)
                {
//                If corner amount is lesser or equal to 6 then this code fills shape
                    if(shapeAngles[i]>=440 && shapeAngles[i]<=640)
                    {
                        rectangle(output,boundRect[i].tl(),boundRect[i].br(),Scalar(0,255,0),5);
                        drawContours(output,conPoly,i,Scalar(0,100,0),FILLED);
                        objectType = "Besgen";
                        putText(frame,objectType,{boundRect[i].x,boundRect[i].y - 5},FONT_HERSHEY_PLAIN,3,Scalar(0,69,255),2);
                    }

                }
                else if (objCor == 6)
                {
//                If corner amount is lesser or equal to 6 then this code fills shape
                    if(shapeAngles[i]>=620 && shapeAngles[i]<=820)
                    {
                        rectangle(output,boundRect[i].tl(),boundRect[i].br(),Scalar(0,255,0),5);
                        drawContours(output,conPoly,i,Scalar(0,200,200),FILLED);
                        objectType = "Altigen";
                        putText(frame,objectType,{boundRect[i].x,boundRect[i].y - 5},FONT_HERSHEY_PLAIN,3,Scalar(0,69,255),2);
                    }

                }
                else if (objCor <=8)
                {
//      ###############################################################################################################################################
        //                If shape has more than 6 corners then finding code uses HoughCircles method

                    if(past_time>1)
                    {
                        cvtColor(circleFrame, circleGray, COLOR_BGR2GRAY);
                        medianBlur(circleGray, circleGray, 5);
                        HoughCircles(circleGray, circles, HOUGH_GRADIENT, 0.01,
                                         100,  // change this value to detect circles with different distances to each other
                                         90, 45, 10, 100 // change the last two parameters
                                    // (min_radius & max_radius) to detect larger circles
                            );
                    }

                        for( size_t i = 0; i < circles.size(); i++ )
                            {
                                Vec3i c = circles[i];
                                Point center = Point(c[0], c[1]);
                                // circle outline
                                int radius = c[2];
                                objectType = "Daire";
                                circle( output, center, radius, Scalar(0,0,255),FILLED,LINE_8);
                                putText(frame,objectType,center,FONT_HERSHEY_PLAIN,3,Scalar(0,69,255),2);
//                                rectangle(output,boundingRect(c[i]).tl(),boundingRect(c[i]).br(),Scalar(0,255,0),5);
                            }
                           objectType="";
//      ###############################################################################################################################################
//                    if(shapeAngles[i]>=700 && shapeAngles[i]<=1020)
//                    {

//                        drawContours(output,conPoly,i,Scalar(0,0,255),FILLED);

//                        double maximum_x = 0;
//                        double maximum_y = 0;
//                        double minimum_x = 0;
//                        double minimum_y = 0;

//                        for(int j=0;j<conPoly[i].size();j++)
//                        {
//                            if(conPoly[i][j].x>maximum_x){
//                                maximum_x = conPoly[i][j].x;
//                            }
//                            else if(conPoly[i][j].x<minimum_x){
//                                minimum_x = conPoly[i][j].x;
//                            }
//                            if(conPoly[i][j].y>maximum_y){
//                                maximum_y = conPoly[i][j].y;
//                            }
//                            else if(conPoly[i][j].y<minimum_y){
//                                minimum_y = conPoly[i][j].y;
//                            }
//                        }

//                        Point center = Point((maximum_x+minimum_x)/2,(maximum_y+minimum_y)/2);


//                        double center_x = abs(conPoly[i][0].x+conPoly[i][(int)conPoly[i].size()/2].x);
//                        double center_y = abs(conPoly[i][0].y+conPoly[i][(int)conPoly[i].size()/2].y);
//                        Point center =Point(center_x/2,center_y/2);
//                        double radius = sqrt(pow(abs(conPoly[i][0].x-center.x),2)+pow(abs(conPoly[i][0].y-center.y),2));
//                        circle(output, center, radius, Scalar(0,0,255),FILLED,LINE_8);
//                        circle(output, center, sqrt(shape_areas[i]/PI), Scalar(0,0,255),FILLED,LINE_8);
//                        cout<<shape_areas[i]<<endl;

//                    }
                }
//            Puts texts of shapes onto them

            }

}

int main()
{
//  This line holds starting time of code
    start_time = time(nullptr);
//    capture variable is for direct photo inputs
//    string capture = samples::findFile("/home/selcuk/SIMTEK/daire.png");
//    VideoCapture class is for capturing frames from webcams
    VideoCapture vid(0);
    vid.set(CAP_PROP_FRAME_WIDTH,640);
    vid.set(CAP_PROP_FRAME_HEIGHT,480);
//    This line creates a trackbar and gives it default value.
    namedWindow("TracksHSV");
    int hueMin=145;
    int hueMax=180;
    int satMin=10;
    int satMax=65;
    int valueMin=0;
    int valueMax=255;
    createTrackbar("hueMin","TracksHSV",&hueMin,360);
    createTrackbar("hueMax","TracksHSV",&hueMax,360);
    createTrackbar("satMin","TracksHSV",&satMin,255);
    createTrackbar("satMax","TracksHSV",&satMax,255);
    createTrackbar("valueMin","TracksHSV",&valueMin,255);
    createTrackbar("valueMax","TracksHSV",&valueMax,255);
    namedWindow("TracksYCbCr");
    int yMin=0;
    int yMax=255;
    int crMin=134;
    int crMax=145;
    int cbMin=120;
    int cbMax=142;
    createTrackbar("YMin","TracksYCbCr",&yMin,255);
    createTrackbar("YMax","TracksYCbCr",&yMax,255);
    createTrackbar("CrMin","TracksYCbCr",&crMin,255);
    createTrackbar("CrMax","TracksYCbCr",&crMax,255);
    createTrackbar("CbMin","TracksYCbCr",&cbMin,255);
    createTrackbar("CbMax","TracksYCbCr",&cbMax,255);



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
//        Blurring image so we can detect shapes better. But not overdoing it because in that case code won't detect anything
        GaussianBlur(frame,imgBlur,Size(3,3),3,0);



//        Cloning Process Materials
        frame_mask_hsv = imgBlur.clone();
        frame_mask_YCbCr = imgBlur.clone();
//        Converting image BGR color space to Grayscale color space
        cvtColor(frame_mask_hsv,frame_mask_hsv,COLOR_BGR2HSV);
        cvtColor(frame_mask_YCbCr,frame_mask_YCbCr,COLOR_BGR2YCrCb);
        Mat maskHSV,maskYCrCb,maskLAB,maskMerge;
        inRange(frame_mask_hsv,Scalar(hueMin,satMin,valueMin),Scalar(hueMax,satMax,valueMax),maskHSV);
        inRange(frame_mask_YCbCr,Scalar(yMin,crMin,cbMin),Scalar(yMax,crMax,cbMax),maskYCrCb);
        imshow("maskHSV",maskHSV);
        imshow("maskYCrCb",maskYCrCb);
        bitwise_and(maskYCrCb,maskHSV,maskMerge);


        Mat canny_output;
        vector<vector<Point> > handContours;
        vector<Vec4i> handHierarchy;


        findContours( maskMerge, handContours, handHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        int index = findBiggestContour(handContours);

        Mat drawing = Mat::zeros( frame.size(), CV_8UC1 );
        drawContours( drawing, handContours, index, Scalar(255), -1, 8, handHierarchy, 0, Point() );

        imshow("drw", drawing);

        imshow("result",maskMerge);


//        Canny filter is detecting edges better
        Canny( drawing, imgCanny, 25, 75);
//        Creating kernel for using it in dilate function. with Dilate we can detect better and easily.
        Mat kernel = getStructuringElement(MORPH_RECT,Size(3,3));
        dilate(imgCanny,imgDilate,kernel);
        imshow("test",imgDilate);
//        Calling getContours function
        getContours(imgDilate,output,circleFrame);

//        Alpha and Beta value is for tranparenting parameters
        beta = (1.0-alpha);
//        addWeighted function is transparenting image
        addWeighted(frame,alpha,output,beta,0.0,imgTransparent);
//        Imshow showing image
        imshow("Trans",imgTransparent);
//     This code defines mouse call back event
         setMouseCallback("HSV", mouseEvent, &frame);
         setMouseCallback("YCBCR", mouseEvent, &frame);
//        This if statement setting fps
        if (waitKey(1000/fps)>=0)
        {
            VideoCapture release(0);
            break;
        }
    }
    return 0;
}
