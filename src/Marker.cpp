#include "Marker.hpp"




Marker::Marker()
{
        _homo3d_ps = cv::Mat::zeros(1,4,CV_64F);
            _homo3d_ps.at<double>(0,0) = OUTPOINT;
            _homo3d_ps.at<double>(0,1) = OUTPOINT;
            _homo3d_ps.at<double>(0,2) = OUTPOINT;
            _homo3d_ps.at<double>(0,3) = OUTPOINT;
        _homo3d_rot = cv::Mat::zeros(1,3,CV_64F);

       _eucl3d_ps = cv::Mat::zeros(1,3,CV_64F);
            _eucl3d_ps.at<double>(0,0) = OUTPOINT;
            _eucl3d_ps.at<double>(0,1) = OUTPOINT;
            _eucl3d_ps.at<double>(0,2) = OUTPOINT;
       _eucl3d_rot = cv::Mat::zeros(1,3,CV_64F);

       infoundcams = 0;

       polygon = new GL::Object(GL_POLYGON); //GL_POLYGON
}

Marker::~Marker()
{
    //dtor
}


