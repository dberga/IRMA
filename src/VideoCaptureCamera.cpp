#include <VideoCaptureCamera.hpp>


  VideoCaptureCamera::~VideoCaptureCamera(){

  }

  VideoCaptureCamera::VideoCaptureCamera(int w, int h, int i,std::string filename){ //for video
        width = w;
        height = h;
        frame = cv::Mat(width, height, CV_8UC3);

        Setup();

        name = "VideoCaptureVideo";
        id = 0;
        strcat (serial,name);
        strcat (serial,(char*)i);


         capture = cv::VideoCapture(filename);

  }


VideoCaptureCamera::VideoCaptureCamera(int w, int h, int i){ //for camera device
    width = w;
    height = h;

    Setup();

    name = "VideoCaptureCamera";
    id = i;
    serial = (char*)malloc(sizeof(char) * 50);
    strcat(serial,name);
    strcat(serial,reinterpret_cast<char*>(&id));

    capture = cv::VideoCapture(id);


}


void VideoCaptureCamera::Setup(){


}

char* VideoCaptureCamera::getCameraSerial()
{
        return serial;


}

char* VideoCaptureCamera::getName(){
    return name;
}
cv::Mat VideoCaptureCamera::getFrame(){
        bool bSuccess = capture.read(frame);

         if (!bSuccess)
            {
                 std::cout << name << "[" << id << "]" << " could not read the current frame"<< std::endl;
            }

        return frame;

}

int VideoCaptureCamera::getID(){
    return id;
}

int VideoCaptureCamera::getCamerasNum(){

    std::cout << "VideoCapture cannot autodetect Cameras Num, please set autodetectnum to false and put the right cnum" << std::endl;
    return 0;
}

void VideoCaptureCamera::free(){
    capture.release();
}
