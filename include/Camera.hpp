
#ifndef CAMERA_H
#define CAMERA_H
#include <inc_CV.h>
class Camera{

    public:
        virtual ~Camera(){}
        virtual void Setup() = 0;
        virtual char* getCameraSerial() = 0;
        virtual char* getName() = 0;
        virtual cv::Mat getFrame() = 0;
        virtual int getID() = 0;
        virtual void free() = 0;



        int width;
        int height;


    protected:
        char* serial;
        char* name;
        int id;
        cv::Mat frame;

};

#endif // CAMERA_H

