#include "IRMA.hpp"

IRMA::IRMA(char* path){

    sysfileconfig.directory = (char*)malloc(sizeof(char) * 50);
        strcpy(sysfileconfig.directory,path);

    sysfileconfig.configpath = (char*)malloc(sizeof(char) * 50); //baseconfig
        strcpy(sysfileconfig.configpath,"");
        strcat(sysfileconfig.configpath,sysfileconfig.directory);
        strcat(sysfileconfig.configpath,"config.xml");

    sysfileconfig.configcampath = (char*)malloc(sizeof(char) * 50); //cameras & callibration config
        strcpy(sysfileconfig.configcampath,"");
        strcat(sysfileconfig.configcampath,sysfileconfig.directory);
        strcat(sysfileconfig.configcampath,"configcameras.xml");

    sysfileconfig.configmarkerpath = (char*)malloc(sizeof(char) * 50); //markers & detector config
        strcpy(sysfileconfig.configmarkerpath,"");
        strcat(sysfileconfig.configmarkerpath,sysfileconfig.directory);
        strcat(sysfileconfig.configmarkerpath,"configmarkers.xml");

        loadConfig();
}

IRMA::IRMA(int width,int height,char* cameratype, bool autodetect, int camerasnum, int patternsnum, int basecam){
    sysbase.width = width;
    sysbase.height = height;
    sysbase.cameratype = ""; sysbase.cameratype.append(cameratype);
    if(autodetect==true){
        if(sysbase.cameratype.compare("uEye") == 0){
            sysbase.camsnum = uEyeCamera::getCamerasNum();
        }else if(sysbase.cameratype.compare("VideoCapture") == 0){

            sysbase.camsnum = VideoCaptureCamera::getCamerasNum();
        }else{
            std::cerr << "IRMA////Camera type is not recognised..." << std::endl;
            sysbase.camsnum = 0;
        }
    }else{
        sysbase.camsnum = camerasnum;
    }
    sysbase.patnum = patternsnum;
    sysbase.basecam = basecam;


    sysfileconfig.directory = (char*)malloc(sizeof(char) * 50);
        strcpy(sysfileconfig.directory,"data/");

    sysfileconfig.configpath = (char*)malloc(sizeof(char) * 50); //baseconfig
        strcpy(sysfileconfig.configpath,"");
        strcat(sysfileconfig.configpath,sysfileconfig.directory);
        strcat(sysfileconfig.configpath,"config.xml");

    sysfileconfig.configcampath = (char*)malloc(sizeof(char) * 50); //cameras & callibration config
        strcpy(sysfileconfig.configcampath,"");
        strcat(sysfileconfig.configcampath,sysfileconfig.directory);
        strcat(sysfileconfig.configcampath,"configcameras.xml");

    sysfileconfig.configmarkerpath = (char*)malloc(sizeof(char) * 50); //markers & detector config
        strcpy(sysfileconfig.configmarkerpath,"");
        strcat(sysfileconfig.configmarkerpath,sysfileconfig.directory);
        strcat(sysfileconfig.configmarkerpath,"configmarkers.xml");

}


IRMA::~IRMA()
{

    cv::destroyAllWindows();
    for(int i=0; i<_cameras.size(); i++){
        _cameras[i]->free();
    }
    _cameras.empty();

    for(int i=0; i<_markers.size(); i++){
        delete _markers[i];
    }
    _markers.empty();

    delete _origin;


    bdetector.empty();
    shapes.clear();
    foundshapes.clear();

    window3D->KillGLWindow();
    std::cout << msg.wParam << std::endl;

}


void IRMA::Config(){
    std::cout << "IRMA////CONFIG////IRMA CONFIG STARTED" << std::endl;
             InitParameters();
             InitCameras();
             loadCalibrationConfig();
             loadCamerasConfig();
             loadDetectorConfig();
             loadPatternsConfig();
             loadOriginConfig();
             AssignMarkers();
             GLInit();
     std::cout << "IRMA////CONFIG////IRMA CONFIG ENDED SUCCESSFULLY" << std::endl;

}

void IRMA::showDevices(){

    if(sysbase.camsnum == 0)
                std::cout << "       No cameras connected"<< std::endl;
    else{
        std::stringstream message;
        message << "IRMA////IRMA DEVICES" << std::endl;

        for(int i=0;i<_cameras.size(); i++){
            message << _cameras[i]->getName();
            message << "[id=";
            message << _cameras[i]->getID();
            message << "]";
            message << "[serial=";
             message << _cameras[i]->getCameraSerial();
            message << "]";
            std::cout << message.str() << std::endl;
            message.str("");
        }
    }
}



void IRMA::WindowsCreate(){

    std::stringstream windowname;
    for(int i=0; i< _cameras.size(); i++){
        windowname << "["<< _cameras[i]->getID() << "]";
        windowname << _cameras[i]->getName();
         cv::namedWindow(windowname.str(), CV_WINDOW_AUTOSIZE);
         windowname.str("");
    }

}

void IRMA::WindowsUpdate(){

    std::stringstream windowname;
    for(int i=0;i<_cameras.size();i++){
        windowname << "["<<_cameras[i]->getID() << "]";
        windowname << _cameras[i]->getName();
        cv::imshow(windowname.str(),sysmomentum._frames_show[i]);
        windowname.str("");
    }
}

void IRMA::changeStatus(int newstatus){
    sysstatus.status = newstatus;
}

void IRMA::InitParameters(){

    for(int i=0; i<sysbase.camsnum; i++){
            sysparameters.camcalibrateparameters.push_back(calibrateparameters());
            sysparameters.camintrinsicparameters.push_back(intrinsicparameters());
            sysparameters.camextrinsicparameters.push_back(extrinsicparameters());
            sysparameters.camrelativeparameters.push_back(relativeparameters());
    }
    for(int i=0; i<sysbase.camsnum; i++){
            sysstatus.camerastatus.push_back(0);
            sysstatus.status = 0;
    }

    for(int i=0; i<sysbase.patnum; i++){
            syspatterns.patterns.push_back(pattern());
    }

    for(int i=0; i<sysbase.camsnum; i++){
            sysmomentum._frames.push_back(cv::Mat::zeros(sysbase.height, sysbase.width, CV_8UC3));
            sysmomentum._frames_undistorted.push_back(cv::Mat::zeros(sysbase.height, sysbase.width, CV_8UC3));
            sysmomentum._frames_binary.push_back(cv::Mat::zeros(sysbase.height, sysbase.width, CV_8UC3));
            sysmomentum._frames_blobbed.push_back(cv::Mat::zeros(sysbase.height, sysbase.width, CV_8UC3));
            sysmomentum._frames_marked.push_back(cv::Mat::zeros(sysbase.height, sysbase.width, CV_8UC3));
            sysmomentum._frames_show.push_back(cv::Mat::zeros(sysbase.height, sysbase.width, CV_8UC3));

    }
    for(int i=0; i<sysbase.camsnum; i++){
            syspoints._blobs.push_back(std::vector<cv::KeyPoint>());
            syspoints._markers_pos_pt.push_back(std::vector<cv::Point>());
            syspoints._markers_rot_pt.push_back(std::vector<double>());
            foundshapes.push_back(std::vector<shape>());
            shapes.push_back(std::vector<shape>());
    }

    _origin = new Marker();


    sysmomentum.drawblobs = true;
    sysmomentum.drawcanny = true;
    sysmomentum.drawmarkers = true;
    sysmomentum.drawmarkers3D = true;
}

void IRMA::InitCameras(){

        for(int i=0; i<sysbase.camsnum; i++){
            if(sysbase.cameratype.compare("uEye")==0){
                _cameras.push_back(new uEyeCamera(sysbase.width,sysbase.height));
            }else if(sysbase.cameratype.compare("VideoCapture")==0){
                _cameras.push_back(new VideoCaptureCamera(sysbase.width,sysbase.height,i+1));
            }else{
                std::cout << "IRMA////CAMERA TYPE"<< sysbase.cameratype<<"IS NOT DEFINED"<< std::endl;
            }
        }


}
void IRMA::AssignMarkers(){

    for(int i=0; i<sysbase.patnum; i++){
            Marker *mark = new Marker();
            mark->pat = syspatterns.patterns[i];
            _markers.push_back(mark);

    }
    for(int i=0; i<sysbase.camsnum; i++){
        for(int j=0; j<sysbase.patnum; j++){
            syspoints._markers_pos_pt[i].push_back({OUTPOINT,OUTPOINT});
            syspoints._markers_rot_pt[i].push_back(0);
        }
    }
     for(int i=0; i<sysbase.camsnum; i++){
            syspoints._origin_pos_pt.push_back({OUTPOINT,OUTPOINT});

    }

    for(int i=0; i<sysbase.patnum; i++){
        syspoints.foundinbasecam.push_back(false);
    }

}

void IRMA::changeCamerasStatus(int newstatus){
    for(int i=0; i< _cameras.size(); i++){
        sysstatus.camerastatus[i] = newstatus;
    }
}

void IRMA::TrackbarsCreate(){
    cv::namedWindow("Trackbars", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("minthreshold", "Trackbars", &sysmarkerparameters.minThreshold, 255);
    cv::createTrackbar("maxthreshold", "Trackbars", &sysmarkerparameters.maxThreshold, 255);
    cv::createTrackbar("aperturesize", "Trackbars", &sysmarkerparameters.aperturesize, 7);
    cv::createTrackbar("kerneldims", "Trackbars", &sysmarkerparameters.kerneldims, 7);
}

void IRMA::GLInit(){
    scenario = new GL::Object(GL_LINES);
    gldraw = new GL::Draw();

    CreateScenario(scenario);
    gldraw->pushObject(scenario);

    for(int m=0;m<_markers.size(); m++){
        CreatePolygon(_markers[m]->polygon, _markers[m]->pat.vertexnum);
        gldraw->pushObject(_markers[m]->polygon);
    }


}

LPWSTR IRMA::charToWideChar(char* str){
	int len = MultiByteToWideChar(CP_ACP, 0, str, -1, NULL, 0);
	wchar_t* w_str = new wchar_t[len];
	MultiByteToWideChar(CP_ACP, 0, str, -1, w_str, len);
	return w_str;
}

void IRMA::Window3DCreate(){


    this->window3D = new GL::OpenGLWindow( 640,
                                    480,
                                    IPL_DEPTH_8U,
                                    charToWideChar("3D Window"),
                                    this->gldraw);

    if (!this->window3D->CreateGLWindow())
	{
		//quit
		std::cout << "error" << std::endl;
	}


}




