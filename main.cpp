#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <ctime>
#include <math.h>
#include <iostream>
#include <Calculations.h>
#include <shapeobjects.h>
#define PI 3.14159265
#define WIDTH 640
#define HEIGHT 480
using namespace cv;
using namespace std;

Mat frame,output,circleFrame,frame_mask_hsv,frame_mask_YCbCr,frame_mask_lab,hand_frame;
Mat maskHSV,maskYCrCb,maskLAB,maskMerge;
Mat imgBlur, imgBilateral, imgCanny,imgDilate,imgGray;
Mat output_approx, imgTransparent, circleGray;
vector<Point2f> boxPts(4);
String objectType;
const int fps =30;
double alpha = 0.6;
double beta;
vector<double> shapeAngles;
time_t current_time,start_time,past_time,scanner_time;
vector<Point> tempPoly;
vector<vector<Point>> conPoly;
vector<Rect> boundRect;
vector<Vec3f> circles;
vector<int> shape_areas;
vector<Point> drawing;
Mat hsvchannel[3],ycrcbchannel[3];
Calculations calculations;
bool Lock = false;
bool Scan = true;
// Yapim asamasinda
//#####################################################################################################################################################
ShapeObjects customizeShapes(VideoCapture vid,Mat drawingFrame, Point fingerTop, ShapeObjects shape)
{
    if(fingerTop.x<shape.getCenter().x+shape.getRadius() && fingerTop.x>shape.getCenter().x-shape.getRadius() &&
            fingerTop.y<shape.getCenter().y+shape.getRadius() && fingerTop.y>shape.getCenter().y-shape.getRadius())
    {
        current_time = time(nullptr);
    }
    else
    {
        scanner_time = time(nullptr);
    }


    past_time = current_time - scanner_time;
    cout<<past_time<<endl<<shape.getCenter();

    if(past_time > 2)
    {
        scanner_time = time(nullptr);
        Lock = !Lock;
        Scan = !Scan;
    }
    if(Lock)
    {
        shape.setCenter(fingerTop);
    }
    circle(drawingFrame, shape.getCenter(), shape.getRadius(), Scalar(255,50,100),FILLED,LINE_8);
    return shape;
}
//#####################################################################################################################################################
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
                    shapeAngles.push_back(calculations.calculate_angle(tempPoly));
                    shape_areas.push_back(area);
                }
        }
    }



            for(int i = 0; i < conPoly.size();i++)
            {
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
    vid.set(CAP_PROP_FRAME_WIDTH,WIDTH);
    vid.set(CAP_PROP_FRAME_HEIGHT,HEIGHT);

//    Index Points of drawing shapes
    Point centerFrame = Point(WIDTH/2,HEIGHT/2);

    ShapeObjects moveableShape(centerFrame,40);

//    This line creates a trackbar and gives it default value.
    namedWindow("TracksHSV");
    int hueMin=140;
    int hueMax=180;
    int satMin=10;
    int satMax=125;
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
    int crMin=129;
    int crMax=155;
    int cbMin=122;
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
        hand_frame = frame.clone();

//    Preprocessing Image
//        Blurring image so we can detect shapes better. But not overdoing it because in that case code won't detect anything
        GaussianBlur(frame,imgBlur,Size(3,3),3,0);



//        Cloning Process Materials
        frame_mask_hsv = imgBlur.clone();
        frame_mask_YCbCr = imgBlur.clone();
//        Converting images colorspaces
        cvtColor(frame_mask_hsv,frame_mask_hsv,COLOR_BGR2HSV);
        cvtColor(frame_mask_YCbCr,frame_mask_YCbCr,COLOR_BGR2YCrCb);
//         Setting masks color channels so we can detect skin easily

        inRange(frame_mask_hsv,Scalar(hueMin,satMin,valueMin),Scalar(hueMax,satMax,valueMax),maskHSV);
        inRange(frame_mask_YCbCr,Scalar(yMin,crMin,cbMin),Scalar(yMax,crMax,cbMax),maskYCrCb);
        imshow("maskHSV",maskHSV);
        imshow("maskYCrCb",maskYCrCb);
//        Merging maskYCrCb and maskHSV so we can get merged mask and with that detect skin
        bitwise_and(maskYCrCb,maskHSV,maskMerge);

//        Declaring vectors for hand contours
        vector<vector<Point>> handContours;
        vector<Vec4i> handHierarchy;
        Point fingerTop;


        Mat kernel = getStructuringElement(MORPH_RECT,Size(5,5));
//        We can find contours of merged mask here then call findBiggestContour method
        erode(maskMerge,maskMerge,kernel);
        for(int i =0;i<2;i++)
        {
            dilate(maskMerge,maskMerge,kernel);
        }
        findContours( maskMerge, handContours, handHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        int index = calculations.findBiggestContour(handContours);
        if(index>-1)
        {
            fingerTop   = *min_element(handContours[index].begin(), handContours[index].end(),[](const Point& lhs, const Point& rhs) {return lhs.y < rhs.y;});
        }
        drawing.push_back(fingerTop);
        if(drawing.size() > 5)
        {
            drawing.erase(drawing.begin());
        }

//        We declare an empty window so we can draw on it.
        Mat hand_draw = Mat::zeros( frame.size(), CV_8UC1 );
        drawContours(hand_draw, handContours, index, Scalar(255), -1, 8, handHierarchy, 0, Point() );
        drawContours(hand_frame, handContours, index, Scalar(255), -1, 8, handHierarchy, 0, Point() );

        imshow("Hand", hand_draw);


//        Canny filter is detecting edges better
        Canny( imgBlur, imgCanny, 25, 75);
//        Creating kernel for using it in dilate function. with Dilate we can detect better and easily.
        kernel = getStructuringElement(MORPH_RECT,Size(3,3));
        dilate(imgCanny,imgDilate,kernel);
//        Calling getContours function
        getContours(imgDilate,output,circleFrame);

//        Alpha and Beta value is for tranparenting parameters
        beta = (1.0-alpha);
//        addWeighted function is transparenting image

        moveableShape = customizeShapes(vid,frame,fingerTop, moveableShape);

        addWeighted(frame,alpha,output,beta,0.0,imgTransparent);
        addWeighted(imgTransparent,alpha,hand_frame,beta,0.0,imgTransparent);
        circle(imgTransparent, fingerTop, 10, Scalar(50,100,255),FILLED,LINE_8);
        for(int i=1; i<drawing.size(); i++)
        {
            line(imgTransparent, drawing[i-1], drawing[i], Scalar(0, 255, 0), 2);
        }

//        Imshow showing image
        imshow("Transparent",imgTransparent);
//        This if statement setting fps
        if (waitKey(1000/fps)>=0)
        {
            VideoCapture release(0);
            break;
        }
    }
    return 0;
}
