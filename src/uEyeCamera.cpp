#include <uEyeCamera.hpp>



char* uEyeCamera::getName() {
    return name;
}

int uEyeCamera::getID(){
    return id;
}


uEyeCamera::uEyeCamera(int w, int h) {
        width = w;
        height = h;
        frame = cv::Mat(height, width, CV_8UC3);

        Setup();

            name = "uEyeCamera";
            serial = cameraInfo.SerNo;
            id = (int)hCam;



}

uEyeCamera::~uEyeCamera() {
        int retInt = is_ExitCamera(hCam);
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }
        delete &hCam;
        delete &frame;
        //todo

}




void uEyeCamera::getFrame(cv::Mat& mat) {
        VOID* pMem;
        int retInt = is_GetImageMem(hCam, &pMem);
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }
//      if (mat.cols == width && mat.rows == height && mat.depth() == 3) {
                memcpy(mat.ptr(), pMem, width * height * 3);
//      } else {
//              throw UeyeException(hCam, -1337);
//      }
}

cv::Mat uEyeCamera::getFrame() {
        this->getFrame(frame);
        return frame;
}

HIDS uEyeCamera::getHIDS() {
        return hCam;
}

void uEyeCamera::Setup(){
         hCam = 0;
        char* ppcImgMem;
        int pid;
        INT nAOISupported = 0;
        double on = 1;

        double empty;
        int retInt = IS_SUCCESS;

        //init
        retInt = is_InitCamera(&hCam, 0); //put the device id to hCam, 0 is void*
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }

        //info
        retInt = is_GetCameraInfo(hCam, &cameraInfo);
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }

        //others
        retInt = is_SetColorMode(hCam, IS_CM_BGR8_PACKED);
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }
        retInt = is_ImageFormat(hCam, IMGFRMT_CMD_GET_ARBITRARY_AOI_SUPPORTED, (void*) &nAOISupported, sizeof(nAOISupported));
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }
        retInt = is_AllocImageMem(hCam, width, height, 24, &ppcImgMem, &pid);
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }
        retInt = is_SetImageMem(hCam, ppcImgMem, pid);
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }

        retInt = is_CaptureVideo(hCam, IS_WAIT);
        if (retInt != IS_SUCCESS) {
                throw UeyeException(hCam, retInt);
        }
}

void uEyeCamera::setAutoGain(bool set) {
    double empty;
    double on = set ? 1 : 0;
    int retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &on, &empty);
    if (retInt != IS_SUCCESS) {
            throw UeyeException(hCam, retInt);
    }
}


char* uEyeCamera::getCameraSerial(){
    return serial;
}


 int uEyeCamera::getCamerasNum(){

    int valor;
    is_GetNumberOfCameras( &valor );
    return valor;

}

void uEyeCamera::free(){
    int retInt = is_ExitCamera(hCam);
    if (retInt != IS_SUCCESS) {
            throw UeyeException(hCam, retInt);
    }
}


