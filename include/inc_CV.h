#ifndef INC_CV_H
#define INC_CV_H


    #include "opencv/cv.h"
    #include "opencv/highgui.h"


    typedef cv::Mat Matrix;
    typedef cv::SimpleBlobDetector BlobDetector;
    typedef cv::Point2f Point2f;
    typedef cv::Point3f Point3f;
    typedef cv::Size Size;
    typedef cv::Point Point;
    typedef cv::KeyPoint KeyPoint;
    typedef cv::Rect Rect;
    typedef cv::Scalar Scalar;

    #define VAR_CV_WINDOW_AUTOSIZE CV_WINDOW_AUTOSIZE
    #define VAR_CV_TERMCRIT_ITER CV_TERMCRIT_ITER
    #define VAR_CV_TERMCRIT_EPS CV_TERMCRIT_EPS
    #define VAR_CV_CALIB_SAME_FOCAL_LENGTH CV_CALIB_SAME_FOCAL_LENGTH
    #define VAR_CV_CALIB_ZERO_TANGENT_DIST CV_CALIB_ZERO_TANGENT_DIST
    #define VAR_CV_64F CV_64F
    #define VAR_CV_BGR2GRAY CV_BGR2GRAY
    #define VAR_CV_RETR_EXTERNAL CV_RETR_EXTERNAL
    #define VAR_CV_CHAIN_APPROX_SIMPLE CV_CHAIN_APPROX_SIMPLE
    #define VAR_CV_RGB CV_RGB
    #define VAR_CV_8UC3 CV_8UC3

/*
findChessboardCorners
undistort
stereoCalibrate
TermCriteria
stereoRectify
drawKeypoints
triangulatePoints
cvtColor
blur
Canny
putText
findContours
approxPolyDP
fabs
line
contourArea
isContourConvex
boundingRect
circle
imshow
namedWindow
createTrackbar
*/

#endif // INC_CV_H


