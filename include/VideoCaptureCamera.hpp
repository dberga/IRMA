

#ifndef VIDEOCAPTURECAMERA_H
#define VIDEOCAPTURECAMERA_H

#include "Camera.hpp"

class VideoCaptureCamera: public Camera{

    public:
        void Setup();
        char* getCameraSerial();
        char* getName();
        cv::Mat getFrame();
        int getID();
        void free();




        VideoCaptureCamera(int w, int h, int i,std::string filename); //for video
        VideoCaptureCamera(int w, int h, int i); //for camera device
        virtual ~VideoCaptureCamera();
        static int getCamerasNum();
    private:
        cv::VideoCapture capture;

};



#endif // VIDEOCAPTURECAMERA_H
