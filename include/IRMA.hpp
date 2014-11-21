#ifndef IRMA_H
#define IRMA_H

#include <stdlib.h>

#include "inc_CV.h"
#include "inc_Drivers.h"

#include "pugiconfig.hpp"
#include "pugixml.hpp"

#include "Camera.hpp"
#include "uEyeCamera.hpp"
#include "uEyeExceptions.h"
#include "VideoCaptureCamera.hpp"

#include "Marker.hpp"

#include "parameters.h"


#include "inc_GL.h"


class IRMA
{
    public:
        //creator / destructor
        IRMA(char* path);
        IRMA(int width,int height,char* cameratype, bool autodetect = true, int camerasnum = 2, int patternsnum = 4, int basecam = 0);
        virtual ~IRMA();
        //Setup
        void Config();
        //Iterate
        void iterate();
        //Draw
        void draw();
        void draw3D();
        //Key Triggers
        void triggerKeys(int key);
        //utils
        void showDevices();
        //GUI
        void WindowsCreate();
        void WindowsUpdate();
        void TrackbarsCreate();
        void Window3DCreate();


        //structed parameters
        IRMAbase sysbase;
        IRMAfileconfig sysfileconfig;
        IRMAmomentum sysmomentum;
        IRMAstatus sysstatus;
        IRMAparameters sysparameters;
        IRMApoints syspoints;
        IRMAworld sysworld;
        IRMApatterns syspatterns;
        IRMAmarkerparameters sysmarkerparameters;
        blobdetectorparameters sysblobdetectorparameters;

        std::vector<Camera *> _cameras;
        std::vector<Marker *> _markers;
        Marker* _origin;

    protected:
        //Config
            int loadConfig();
            void InitParameters();
            void InitCameras();
            void AssignMarkers();
            void GLInit();
            int loadCalibrationConfig();
            int loadCamerasConfig();
            int loadDetectorConfig();
            int loadPatternsConfig();
            int loadOriginConfig();
            int saveCamerasIntrinsicConfig();
            int saveCamerasExtrinsicConfig();
            int saveCamerasRelativeConfig();
            int saveOriginConfig();
        //iterate camera caption
            void getFrames();
            void getFramesUndistorted();
        //draw
            void drawFrames();
            void drawFramesUndistorted();
            void drawBlobs();
            void drawMarkers();
            void drawCanny();
            void drawMarkers3D();
            void drawOrigin();
        //triggerKeys
            void triggerStereoCalibrate();
            void triggerStereoRectify();
            void triggerOrigin3D();
            void changeStatus(int newstatus);
            void changeCamerasStatus(int newstatus);
            void triggerDrawCanny();
            void triggerDrawBlobs();
            void triggerDrawMarkers();
            void triggerDrawMarkers3D();

        //Stereo Vision functions
            void CalibCaptureBoard();
            void CalibEraseBadCaptures();
            void StereoCalibrateCameras(int camreference);
            void StereoRectifyCameras(int camreference);
            void TriangulateMarkerPoints(int camreference, int marker);
            void TriangulateOrigin(int camreference);
            void setMarkersHomoPointToEuclideanPoint(int marker);
            void setOriginHomoPointToEuclideanPoint();
            void set3DWorld(int marker);
        //Detector functions
            void detectBlobs();
            void preProcess();
            void detectMarkers();
                 void getShapes();
                 double getCosine(cv::Point pt1, cv::Point pt2, cv::Point pt0);
                 void getMarkers();
            void detectOrigin();
                void getOrigin();
            void detectMarkers3D();
        //gl functions
            void CreateScenario(GL::Object* obj);
            void CreatePolygon(GL::Object* obj,int vertnum );


        //other utils
            std::string double_a_string(double x);
            LPWSTR charToWideChar(char* str);

    private:

        board sysboard;

        pugi::xml_document docconfig;
        pugi::xml_document doccamconfig;
        pugi::xml_document docmarkerconfig;

        cv::SimpleBlobDetector bdetector;
        std::vector<std::vector<shape>> shapes; //selected shapes corresponding to the markers
        std::vector<std::vector<shape>> foundshapes; //total found shapes

        //gl
        GL::Draw* gldraw;
        GL::OpenGLWindow* window3D;
        GL::Object* scenario;
        MSG msg;
};

#endif // IRMA_H
