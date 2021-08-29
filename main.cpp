#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <ctime>
#include <math.h>
#include <iostream>
#include <Calculations.h>
#include <shapeobjects.h>
#include <thread>
#define PI 3.14159265
#define WIDTH 640
#define HEIGHT 480
using namespace cv;
using namespace std;

Mat frame,output,circleFrame,frame_mask_hsv,frame_mask_YCbCr,frame_mask_lab,hand_frame,output_approx,
                                                                            imgTransparent, circleGray,imgBlur, imgBilateral, imgCanny,imgDilate,imgGray,maskHSV,maskYCrCb,maskLAB,maskMerge;
String objectType;
const int fps =30;
double alpha = 0.6;
double beta, textSize;
vector<double> shapeAngles;
time_t current_time,start_time,past_time,scanner_time;
vector<Point> tempPoly,drawing;
vector<vector<Point>> conPoly;
vector<Rect> boundRect;
vector<Vec3f> circles;
vector<int> shape_areas;
vector<ShapeObjects> customObjectList;
Calculations calculations;
String text;
char key;

//Return Circle or Rectangle that program should draw on screen and make its adjustments
ShapeObjects customizeShapes(VideoCapture vid,Mat drawingFrame, Point fingerTop, ShapeObjects shape)
{
//    If person's finger's top is in the borders of shape then activate lock timer. Else reset timers.
    if(fingerTop.x<shape.getCenter().x+shape.getRadius() && fingerTop.x>shape.getCenter().x-shape.getRadius() &&
            fingerTop.y<shape.getCenter().y+shape.getRadius() && fingerTop.y>shape.getCenter().y-shape.getRadius())
    {
        shape.startCurrentTime();
    }
    else
    {
        shape.startCurrentTime();
        shape.startStartTime();
    }
// Calculate the time that finger spent in the borders.
    past_time = shape.getCurrentTime() - shape.getStartTime();
// If past time is more than 2 then lock if it is unlocked or unlock if it is locked
    if(past_time > 2)
    {
        shape.startStartTime();
        shape.setLock(true);
        shape.unlockShape(fingerTop);
        shape.setPastFrame(fingerTop);
    }
//    If locked then change center point
    if(shape.getLock())
    {
        shape.setCenter(fingerTop);
    }
//    If object type is Circle then draw circle as given attributes
    if(shape.getObjectType()=="circle")
    {
        circle(drawingFrame, shape.getCenter(), shape.getRadius(), Scalar(shape.getBlue(),shape.getGreen(),shape.getRed()),FILLED,LINE_8);
    }
//    If object type is Rectangle then draw rectangle and write the input inside the rectangle as given attributes
    else if(shape.getObjectType()=="rectangle")
    {
        Point topLeft = Point(shape.getCenter().x-shape.getRadius(),shape.getCenter().y-shape.getRadius());
        Point bottomRight = Point(shape.getCenter().x+shape.getRadius(),shape.getCenter().y+shape.getRadius());
        rectangle(drawingFrame,topLeft,bottomRight,Scalar(shape.getBlue(),shape.getGreen(),shape.getRed()),-1);
//        Adaptive font sizes. Maximum font size is 2.
        if((shape.getRadius()*2)/53 <= 2)
        {
            textSize = (shape.getRadius()*2)/53;
        }
        else
        {
            textSize = 2;
        }
        putText(frame,shape.getText(),Point(topLeft.x,shape.getCenter().y + (textSize/2)),FONT_HERSHEY_PLAIN,textSize,Scalar(0,69,255),2);
    }
    return shape;
}
// Shape detector method
void getShapes(Mat output_canny,Mat output,Mat circleFrame)
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
                                //            Write shape type onto shape.
                                putText(frame,objectType,center,FONT_HERSHEY_PLAIN,3,Scalar(0,69,255),2);
                            }
                           objectType="";
                }
            }
}
int main()
{
//  This line holds starting time of code
    start_time = time(nullptr);
//    VideoCapture class is for capturing frames from webcams
    VideoCapture vid(0);
    vid.set(CAP_PROP_FRAME_WIDTH,WIDTH);
    vid.set(CAP_PROP_FRAME_HEIGHT,HEIGHT);
//    Index Points of drawing shapes
    Point centerFrame = Point(WIDTH/2,HEIGHT/6);
//    This line creates a trackbar and gives it default value.
    int hueMin=140;
    int hueMax=180;
    int satMin=10;
    int satMax=125;
    int valueMin=0;
    int valueMax=255;
    int yMin=0;
    int yMax=255;
    int crMin=129;
    int crMax=155;
    int cbMin=122;
    int cbMax=142;
    namedWindow("TracksShapeFeatures");
    int blue=0;
    int red=0;
    int green=0;
    int radius = 40;
    createTrackbar("Blue","TracksShapeFeatures",&blue,255);
    createTrackbar("Green","TracksShapeFeatures",&green,255);
    createTrackbar("Red","TracksShapeFeatures",&red,255);
    createTrackbar("Radius","TracksShapeFeatures",&radius,255);
//    This if statement is checking if the code took frame from webcam or not if not code quitting.
    if (!vid.isOpened())
    {
        return -1;
    }
//     This while loop is reading each frame from camera and using it
    while(vid.read(frame))
    {
//        output is clone of frame so we can do some changes on output and transparenting it on frame
        output = frame.clone();
        circleFrame = frame.clone();
        hand_frame = frame.clone();
//        Key variable holds the key pressed.
        key = waitKey(1000/fps)&0XFF;
//        If pressed key is s then create circle with given attributes.
        if(key == 's')
        {
            customObjectList.push_back(ShapeObjects(centerFrame,radius,"circle",blue,red,green,""));
        }
//        If pressed key is d then create rectangle with given attributes and input.
        if(key == 'd')
        {
            cout << "Enter Title: ";
            getline(cin, text);
            customObjectList.push_back(ShapeObjects(centerFrame,radius,"rectangle",blue,red,green,text));
        }
//        If pressed key is r then remove first shape.
        if(key == 'r')
        {
            if(customObjectList.size()>0)
            {
                customObjectList.erase(customObjectList.begin());
            }
        }
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
//        Merging maskYCrCb and maskHSV so we can get merged mask and with that detect skin
        bitwise_and(maskYCrCb,maskHSV,maskMerge);
//        Declaring vectors for hand contours
        vector<vector<Point>> handContours;
        vector<Vec4i> handHierarchy;
        Point fingerTop;
//        Eroding then dilating merged mask so we can find better results
        Mat kernel = getStructuringElement(MORPH_RECT,Size(5,5));
        erode(maskMerge,maskMerge,kernel);
        for(int i =0;i<2;i++)
        {
            dilate(maskMerge,maskMerge,kernel);
        }
//        Finding contours of skins so we can find biggest contour and find top of finger with it.
        findContours(maskMerge, handContours, handHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
//        Calculating biggest contour index so we can find hand better.
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
//        Canny filter is detecting edges better
        Canny( imgBlur, imgCanny, 25, 75);
//        Creating kernel for using it in dilate function. with Dilate we can detect better and easily.
        kernel = getStructuringElement(MORPH_RECT,Size(3,3));
        dilate(imgCanny,imgDilate,kernel);
//        Calling getShapes function in thread so we can calculate shapes while putting moveable objects on screen.
        thread shapeThread(getShapes,imgDilate,output,circleFrame);
//        Alpha and Beta value is for tranparenting parameters
        beta = (1.0-alpha);
//        Draws moveable rectangles and circles on screen
        for(int i = 0;i<customObjectList.size();i++)
        {
            customObjectList[i] = customizeShapes(vid,frame,fingerTop, customObjectList[i]);
        }
//        Program needs join function here because below code is drawing and writing shapes so program needs to find all the thing that it should draw.
        shapeThread.join();
//        addWeighted function is transparenting image
        addWeighted(frame,alpha,output,beta,0.0,imgTransparent);
        addWeighted(imgTransparent,alpha,hand_frame,beta,0.0,imgTransparent);
//        Draws circle on finger
        circle(imgTransparent, fingerTop, 10, Scalar(50,100,255),FILLED,LINE_8);
//        Draws green line before hand.
        for(int i=1; i<drawing.size(); i++)
        {
            line(imgTransparent, drawing[i-1], drawing[i], Scalar(0, 255, 0), 2);
        }
//        Imshow showing image
        imshow("Transparent",imgTransparent);
//        This if statement quits if you press q.
        if (key =='q')
        {
            VideoCapture release(0);
            break;
        }
    }
    return 0;
}
