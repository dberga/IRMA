
#include <IRMA.hpp>




void IRMA::triggerKeys(int key){

    switch (key){
        case 99: //c
            if(sysstatus.status == 0)
                triggerStereoCalibrate();
            break;
        case 114: //r
            if(sysstatus.status == 1 || sysstatus.status == 2)
                triggerStereoRectify();
            break;
        case 111: //o
            if(sysstatus.status == 3)
                triggerOrigin3D();
            break;
        case 49: //1
                triggerDrawCanny();
            break;
        case 50: //2
                triggerDrawBlobs();
            break;
         case 51: //3
                triggerDrawMarkers();
            break;
         case 52: //4
                triggerDrawMarkers3D();
            break;
    }
}

void IRMA::triggerStereoCalibrate(){

        CalibCaptureBoard();
        CalibEraseBadCaptures();

        int sum = 0;
        for(int i=0; i< _cameras.size(); i++){
            if(sysstatus.camerastatus[i] > 0)
                sum++;
        }

        if(sum ==_cameras.size()){
            StereoCalibrateCameras(sysbase.basecam);
            saveCamerasIntrinsicConfig();
            changeStatus(1);
            changeCamerasStatus(1);
            saveCamerasExtrinsicConfig();
            changeStatus(2);
            changeCamerasStatus(2);
            std::cout << "IRMA////STEREOCALIBRATION SUCCESSFUL"<< std::endl;
        }else{
            std::cout << "IRMA////CAPTURE MADE"<< std::endl;
        }
}

void IRMA::triggerStereoRectify(){

        StereoRectifyCameras(sysbase.basecam);
        saveCamerasRelativeConfig();
        changeStatus(3);
        changeCamerasStatus(3);
        std::cout << "IRMA////STEREORECTIFICATION SUCCESSFUL"<< std::endl;
}

void IRMA::triggerOrigin3D(){
        detectOrigin();
        if(_origin->infoundcams == -1)
            std::cout << "IRMA////ORIGIN ALREADY FOUND"    << std::endl;
        else
            std::cout << "IRMA////ORIGIN NOT FOUND"    << std::endl;
}

void IRMA::triggerDrawCanny(){
    if(sysmomentum.drawcanny == true)
        sysmomentum.drawcanny = false,
        std::cout << "IRMA////DISABLED DRAW CANNY & BLUR " <<  std::endl;
    else
        sysmomentum.drawcanny = true,
        std::cout << "IRMA////ENABLED DRAW CANNY & BLUR " <<  std::endl;


}

void IRMA::triggerDrawBlobs(){
    if(sysmomentum.drawblobs == true)
        sysmomentum.drawblobs = false,
        std::cout << "IRMA////DISABLED DRAW BLOBS " <<  std::endl;
    else
        sysmomentum.drawblobs = true,
        std::cout << "IRMA////ENABLED DRAW BLOBS " <<  std::endl;


}

void IRMA::triggerDrawMarkers(){
    if(sysmomentum.drawmarkers == true)
        sysmomentum.drawmarkers = false,
        std::cout << "IRMA////DISABLED DRAW MARKERS " <<  std::endl;
    else
        sysmomentum.drawmarkers = true,
        std::cout << "IRMA////ENABLED DRAW MARKERS " <<  std::endl;


}

void IRMA::triggerDrawMarkers3D(){
    if(sysmomentum.drawmarkers3D == true)
        sysmomentum.drawmarkers3D = false,
        std::cout << "IRMA////DISABLED DRAW MARKERS 3D COORDINATES " <<  std::endl;
    else
        sysmomentum.drawmarkers3D = true,
        std::cout << "IRMA////ENABLED DRAW MARKERS 3D COORDINATES " <<  std::endl;


}
