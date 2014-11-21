#ifndef UEYECAMERA_H
#define UEYECAMERA_H

#include "Camera.hpp"
#include "inc_CV.h"
#include "inc_Drivers.h"
#include "uEyeExceptions.h"


class uEyeCamera: public Camera{
    public:

        void Setup();
        char* getCameraSerial();
        char* getName();
        cv::Mat getFrame();
        int getID();
        void free();




        uEyeCamera(int w, int h);
        virtual ~uEyeCamera();

        HIDS getHIDS();
        void setAutoGain(bool set=true);
        static int getCamerasNum();
        void getFrame(cv::Mat& mat);
    private:
        HIDS hCam; //camera handler -> should be the same as the DevId
        CAMINFO cameraInfo; //info de la camera


};




#endif // UEYECAMERA_H
