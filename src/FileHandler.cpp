#include "IRMA.hpp"


int IRMA::loadConfig(){

    if (!docconfig.load_file(sysfileconfig.configpath))
	{
		std::cerr << "IRMA////LOADCONFIG////Document config.xml does not exist creating a new one..." << std::endl;
        //return -1;
        docconfig.save_file(sysfileconfig.configpath);
        this->loadConfig(); //recursive -> go to the else
	}else{
        pugi::xml_node root_node;
        if (!(root_node = docconfig.child("base")))
        {
            std::cerr << "IRMA////LOADCONFIG////Document has not a valid config. remaking xml nodes..." << std::endl;

             docconfig.append_child("base");
                docconfig.child("base").append_attribute("width")= "640";
                docconfig.child("base").append_attribute("height")= "480";
                docconfig.child("base").append_attribute("cameratype")= "uEye";
                docconfig.child("base").append_attribute("autodetectnum")= "true";
                docconfig.child("base").append_attribute("cnum")= "2";
                docconfig.child("base").append_attribute("pnum")= "4";
                docconfig.child("base").append_attribute("basecam")= "0";
            docconfig.save_file(sysfileconfig.configpath);
            //return -2;
            this->loadConfig();
        }else{
                sysbase.width = root_node.attribute("width").as_int();
                sysbase.height = root_node.attribute("height").as_int();
                sysbase.cameratype = root_node.attribute("cameratype").as_string();
                if(root_node.attribute("autodetectnum").as_bool()==true){
                    if(sysbase.cameratype.compare("uEye") == 0){
                        sysbase.camsnum = uEyeCamera::getCamerasNum();
                    }else if(sysbase.cameratype.compare("VideoCapture") == 0){
                        sysbase.camsnum = VideoCaptureCamera::getCamerasNum();
                    }else{
                        std::cerr << "IRMA////LOADCONFIG////Camera type is not recognised..." << std::endl;
                        sysbase.camsnum = 0;
                    }
                }else{
                    sysbase.camsnum = root_node.attribute("cnum").as_int();
                }
                sysbase.patnum = root_node.attribute("pnum").as_int();
                sysbase.basecam = root_node.attribute("basecam").as_int();
        }
         return 0;
	}
}

int IRMA::loadCalibrationConfig(){
    if (!doccamconfig.load_file(sysfileconfig.configcampath))
    {
            std::cerr << "IRMA////LOADCALIBSCONFIG////Document configcameras.xml does not exist, creating a new one..." << std::endl;
            //return -1
            doccamconfig.save_file(sysfileconfig.configcampath);
            this->loadCalibrationConfig();
    }else{
            pugi::xml_node root_node;
            if(!(root_node = doccamconfig.child("calibration"))){

                std::cerr << "IRMA////LOADCALIBSCONFIG////Document configcameras.xml has bad config, creating a new one..." << std::endl;
                doccamconfig.append_child("calibration");
                doccamconfig.child("calibration").append_attribute("ncaptures")="10";
                doccamconfig.child("calibration").append_child("board");
                doccamconfig.child("calibration").child("board").append_attribute("width")="9";
                doccamconfig.child("calibration").child("board").append_attribute("height")="6";
                doccamconfig.child("calibration").child("board").append_attribute("entitysize")="0.0f";
                doccamconfig.save_file(sysfileconfig.configcampath);
                //return -2
                this->loadCalibrationConfig();
            }else{
                sysboard.ncaptures = root_node.attribute("ncaptures").as_int();
                sysboard.width = root_node.child("board").attribute("width").as_int();
                sysboard.height = root_node.child("board").attribute("height").as_int();
                sysboard.entitysize = root_node.child("board").attribute("entitysize").as_double();
            }

    }
    return 0;
}

int IRMA::loadCamerasConfig(){
        if (!doccamconfig.load_file(sysfileconfig.configcampath))
        {
            std::cerr << "IRMA////LOADCAMERASCONFIG////Document configcameras.xml does not exist, creating a new one..." << std::endl;
            doccamconfig.save_file(sysfileconfig.configcampath);
            this->loadCamerasConfig();
            //return -1;
        }else{
            pugi::xml_node root_node;
            if(!(root_node = doccamconfig.child("cameras"))){
                std::cout << "IRMA////LOADCAMERASCONFIG////There's no cameras tag, remaking nodes..." << std::endl;
                doccamconfig.append_child("cameras");
                doccamconfig.save_file(sysfileconfig.configcampath);
                //return -2
                this->loadCamerasConfig();

            }else{

                for(int i=0; i<sysbase.camsnum; i++){
                    pugi::xml_node camnode;
                     if(camnode= root_node.find_child_by_attribute("camera","serialid",_cameras[i]->getCameraSerial())){
                        sysstatus.camerastatus[i] = camnode.attribute("status").as_int();

                        //case 1 already calibrated +intrinsic parameters
                            cv::Mat distCoeffs;
                            cv::Mat cameramatrix = cv::Mat(3,3,CV_64F);
                            cv::Mat distCoeffsOther;
                            cv::Mat cameramatrixOther = cv::Mat(3,3,CV_64F);
                        //case 2 already stereocalibrated +extrinsic parameters
                            cv::Mat R = cv::Mat(3,3,CV_64F);
                            cv::Mat T = cv::Mat(3,1,CV_64F);
                            cv::Mat E = cv::Mat(3,3,CV_64F);
                            cv::Mat F = cv::Mat(3,3,CV_64F);
                        //case 3 already rectified +relative parameters
                            cv::Mat R1 = cv::Mat(3,3,CV_64F);
                            cv::Mat R2 = cv::Mat(3,3,CV_64F);
                            cv::Mat P1 = cv::Mat(3,4,CV_64F);
                            cv::Mat P2 = cv::Mat(3,4,CV_64F);
                            cv::Mat Q = cv::Mat(4,4,CV_64F);


                        switch (sysstatus.camerastatus[i]){
                            case 1:
                                if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k6").empty()){
                                        distCoeffs = cv::Mat(1,8,CV_64F);
                                        distCoeffs.at<double>(0,7)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k6").as_double();
                                        distCoeffs.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").as_double();
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").empty()){
                                        distCoeffs = cv::Mat(1,7,CV_64F);
                                        distCoeffs.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").as_double();
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").empty()){
                                        distCoeffs = cv::Mat(1,6,CV_64F);
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").empty()){
                                        distCoeffs = cv::Mat(1,5,CV_64F);
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else{
                                    distCoeffs = cv::Mat(1,4,CV_64F);
                                }

                                        distCoeffs.at<double>(0,3)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("p2").as_double();
                                        distCoeffs.at<double>(0,2)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("p1").as_double();
                                        distCoeffs.at<double>(0,1)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k2").as_double();
                                        distCoeffs.at<double>(0,0)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k1").as_double();

                                sysparameters.camintrinsicparameters[i].distCoeffs = distCoeffs.clone();

                                if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k6").empty()){
                                        distCoeffsOther = cv::Mat(1,8,CV_64F);
                                        distCoeffsOther.at<double>(0,7)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k6").as_double();
                                        distCoeffsOther.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").as_double();
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").empty()){
                                        distCoeffsOther = cv::Mat(1,7,CV_64F);
                                        distCoeffsOther.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").as_double();
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").empty()){
                                        distCoeffsOther = cv::Mat(1,6,CV_64F);
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").empty()){
                                        distCoeffsOther = cv::Mat(1,5,CV_64F);
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else{
                                    distCoeffsOther = cv::Mat(1,4,CV_64F);
                                }

                                        distCoeffsOther.at<double>(0,3)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("p2").as_double();
                                        distCoeffsOther.at<double>(0,2)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("p1").as_double();
                                        distCoeffsOther.at<double>(0,1)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k2").as_double();
                                        distCoeffsOther.at<double>(0,0)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k1").as_double();

                                sysparameters.camintrinsicparameters[i].distCoeffsOther = distCoeffsOther.clone();



                                cameramatrix.at<double>(0,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r00").as_double();
                                cameramatrix.at<double>(0,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r01").as_double();
                                cameramatrix.at<double>(0,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r02").as_double();
                                cameramatrix.at<double>(1,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r10").as_double();
                                cameramatrix.at<double>(1,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r11").as_double();
                                cameramatrix.at<double>(1,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r12").as_double();
                                cameramatrix.at<double>(2,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r20").as_double();
                                cameramatrix.at<double>(2,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r21").as_double();
                                cameramatrix.at<double>(2,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r22").as_double();

                                sysparameters.camintrinsicparameters[i].cameramatrix = cameramatrix.clone();

                                cameramatrixOther.at<double>(0,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r00").as_double();
                                cameramatrixOther.at<double>(0,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r01").as_double();
                                cameramatrixOther.at<double>(0,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r02").as_double();
                                cameramatrixOther.at<double>(1,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r10").as_double();
                                cameramatrixOther.at<double>(1,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r11").as_double();
                                cameramatrixOther.at<double>(1,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r12").as_double();
                                cameramatrixOther.at<double>(2,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r20").as_double();
                                cameramatrixOther.at<double>(2,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r21").as_double();
                                cameramatrixOther.at<double>(2,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r22").as_double();

                                sysparameters.camintrinsicparameters[i].cameramatrixOther = cameramatrixOther.clone();

                                sysstatus.status = 1;
                                break;

                            case 2:
                                if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k6").empty()){
                                        distCoeffs = cv::Mat(1,8,CV_64F);
                                        distCoeffs.at<double>(0,7)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k6").as_double();
                                        distCoeffs.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").as_double();
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").empty()){
                                        distCoeffs = cv::Mat(1,7,CV_64F);
                                        distCoeffs.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").as_double();
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").empty()){
                                        distCoeffs = cv::Mat(1,6,CV_64F);
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").empty()){
                                        distCoeffs = cv::Mat(1,5,CV_64F);
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else{
                                    distCoeffs = cv::Mat(1,4,CV_64F);
                                }

                                        distCoeffs.at<double>(0,3)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("p2").as_double();
                                        distCoeffs.at<double>(0,2)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("p1").as_double();
                                        distCoeffs.at<double>(0,1)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k2").as_double();
                                        distCoeffs.at<double>(0,0)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k1").as_double();

                                sysparameters.camintrinsicparameters[i].distCoeffs = distCoeffs.clone();

                                if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k6").empty()){
                                        distCoeffsOther = cv::Mat(1,8,CV_64F);
                                        distCoeffsOther.at<double>(0,7)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k6").as_double();
                                        distCoeffsOther.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").as_double();
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").empty()){
                                        distCoeffsOther = cv::Mat(1,7,CV_64F);
                                        distCoeffsOther.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").as_double();
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").empty()){
                                        distCoeffsOther = cv::Mat(1,6,CV_64F);
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").empty()){
                                        distCoeffsOther = cv::Mat(1,5,CV_64F);
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else{
                                    distCoeffsOther = cv::Mat(1,4,CV_64F);
                                }

                                        distCoeffsOther.at<double>(0,3)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("p2").as_double();
                                        distCoeffsOther.at<double>(0,2)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("p1").as_double();
                                        distCoeffsOther.at<double>(0,1)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k2").as_double();
                                        distCoeffsOther.at<double>(0,0)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k1").as_double();

                                sysparameters.camintrinsicparameters[i].distCoeffsOther = distCoeffsOther.clone();



                                cameramatrix.at<double>(0,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r00").as_double();
                                cameramatrix.at<double>(0,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r01").as_double();
                                cameramatrix.at<double>(0,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r02").as_double();
                                cameramatrix.at<double>(1,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r10").as_double();
                                cameramatrix.at<double>(1,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r11").as_double();
                                cameramatrix.at<double>(1,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r12").as_double();
                                cameramatrix.at<double>(2,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r20").as_double();
                                cameramatrix.at<double>(2,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r21").as_double();
                                cameramatrix.at<double>(2,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r22").as_double();

                                sysparameters.camintrinsicparameters[i].cameramatrix = cameramatrix.clone();

                                cameramatrixOther.at<double>(0,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r00").as_double();
                                cameramatrixOther.at<double>(0,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r01").as_double();
                                cameramatrixOther.at<double>(0,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r02").as_double();
                                cameramatrixOther.at<double>(1,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r10").as_double();
                                cameramatrixOther.at<double>(1,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r11").as_double();
                                cameramatrixOther.at<double>(1,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r12").as_double();
                                cameramatrixOther.at<double>(2,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r20").as_double();
                                cameramatrixOther.at<double>(2,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r21").as_double();
                                cameramatrixOther.at<double>(2,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r22").as_double();

                                sysparameters.camintrinsicparameters[i].cameramatrixOther = cameramatrixOther.clone();

                                R.at<double>(0,0) = camnode.child("extrinsicparameters").child("R").attribute("r00").as_double();
                                R.at<double>(0,1) = camnode.child("extrinsicparameters").child("R").attribute("r01").as_double();
                                R.at<double>(0,2) = camnode.child("extrinsicparameters").child("R").attribute("r02").as_double();
                                R.at<double>(1,0) = camnode.child("extrinsicparameters").child("R").attribute("r10").as_double();
                                R.at<double>(1,1) = camnode.child("extrinsicparameters").child("R").attribute("r11").as_double();
                                R.at<double>(1,2) = camnode.child("extrinsicparameters").child("R").attribute("r12").as_double();
                                R.at<double>(2,0) = camnode.child("extrinsicparameters").child("R").attribute("r20").as_double();
                                R.at<double>(2,1) = camnode.child("extrinsicparameters").child("R").attribute("r21").as_double();
                                R.at<double>(2,2) = camnode.child("extrinsicparameters").child("R").attribute("r22").as_double();
                                sysparameters.camextrinsicparameters[i].R = R.clone();

                                T.at<double>(0,0) = camnode.child("extrinsicparameters").child("T").attribute("r00").as_double();
                                T.at<double>(1,0) = camnode.child("extrinsicparameters").child("T").attribute("r01").as_double();
                                T.at<double>(2,0) = camnode.child("extrinsicparameters").child("T").attribute("r02").as_double();
                                sysparameters.camextrinsicparameters[i].T = T.clone();

                                E.at<double>(0,0) = camnode.child("extrinsicparameters").child("E").attribute("r00").as_double();
                                E.at<double>(0,1) = camnode.child("extrinsicparameters").child("E").attribute("r01").as_double();
                                E.at<double>(0,2) = camnode.child("extrinsicparameters").child("E").attribute("r02").as_double();
                                E.at<double>(1,0) = camnode.child("extrinsicparameters").child("E").attribute("r10").as_double();
                                E.at<double>(1,1) = camnode.child("extrinsicparameters").child("E").attribute("r11").as_double();
                                E.at<double>(1,2) = camnode.child("extrinsicparameters").child("E").attribute("r12").as_double();
                                E.at<double>(2,0) = camnode.child("extrinsicparameters").child("E").attribute("r20").as_double();
                                E.at<double>(2,1) = camnode.child("extrinsicparameters").child("E").attribute("r21").as_double();
                                E.at<double>(2,2) = camnode.child("extrinsicparameters").child("E").attribute("r22").as_double();
                                sysparameters.camextrinsicparameters[i].E = E.clone();

                                F.at<double>(0,0) = camnode.child("extrinsicparameters").child("F").attribute("r00").as_double();
                                F.at<double>(0,1) = camnode.child("extrinsicparameters").child("F").attribute("r01").as_double();
                                F.at<double>(0,2) = camnode.child("extrinsicparameters").child("F").attribute("r02").as_double();
                                F.at<double>(1,0) = camnode.child("extrinsicparameters").child("F").attribute("r10").as_double();
                                F.at<double>(1,1) = camnode.child("extrinsicparameters").child("F").attribute("r11").as_double();
                                F.at<double>(1,2) = camnode.child("extrinsicparameters").child("F").attribute("r12").as_double();
                                F.at<double>(2,0) = camnode.child("extrinsicparameters").child("F").attribute("r20").as_double();
                                F.at<double>(2,1) = camnode.child("extrinsicparameters").child("F").attribute("r21").as_double();
                                F.at<double>(2,2) = camnode.child("extrinsicparameters").child("F").attribute("r22").as_double();
                                sysparameters.camextrinsicparameters[i].F = F.clone();

                                sysstatus.status = 2;
                                break;
                            case 3:
                                if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k6").empty()){
                                        distCoeffs = cv::Mat(1,8,CV_64F);
                                        distCoeffs.at<double>(0,7)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k6").as_double();
                                        distCoeffs.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").as_double();
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").empty()){
                                        distCoeffs = cv::Mat(1,7,CV_64F);
                                        distCoeffs.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k5").as_double();
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").empty()){
                                        distCoeffs = cv::Mat(1,6,CV_64F);
                                        distCoeffs.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k4").as_double();
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").empty()){
                                        distCoeffs = cv::Mat(1,5,CV_64F);
                                        distCoeffs.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k3").as_double();
                                }else{
                                    distCoeffs = cv::Mat(1,4,CV_64F);
                                }

                                        distCoeffs.at<double>(0,3)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("p2").as_double();
                                        distCoeffs.at<double>(0,2)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("p1").as_double();
                                        distCoeffs.at<double>(0,1)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k2").as_double();
                                        distCoeffs.at<double>(0,0)= camnode.child("intrinsicparameters").child("distCoeffs").attribute("k1").as_double();

                                sysparameters.camintrinsicparameters[i].distCoeffs = distCoeffs.clone();

                                if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k6").empty()){
                                        distCoeffsOther = cv::Mat(1,8,CV_64F);
                                        distCoeffsOther.at<double>(0,7)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k6").as_double();
                                        distCoeffsOther.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").as_double();
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").empty()){
                                        distCoeffsOther = cv::Mat(1,7,CV_64F);
                                        distCoeffsOther.at<double>(0,6)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k5").as_double();
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").empty()){
                                        distCoeffsOther = cv::Mat(1,6,CV_64F);
                                        distCoeffsOther.at<double>(0,5)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k4").as_double();
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else if(!camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").empty()){
                                        distCoeffsOther = cv::Mat(1,5,CV_64F);
                                        distCoeffsOther.at<double>(0,4)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k3").as_double();
                                }else{
                                    distCoeffsOther = cv::Mat(1,4,CV_64F);
                                }

                                        distCoeffsOther.at<double>(0,3)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("p2").as_double();
                                        distCoeffsOther.at<double>(0,2)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("p1").as_double();
                                        distCoeffsOther.at<double>(0,1)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k2").as_double();
                                        distCoeffsOther.at<double>(0,0)= camnode.child("intrinsicparameters").child("distCoeffsOther").attribute("k1").as_double();

                                sysparameters.camintrinsicparameters[i].distCoeffsOther = distCoeffsOther.clone();



                                cameramatrix.at<double>(0,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r00").as_double();
                                cameramatrix.at<double>(0,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r01").as_double();
                                cameramatrix.at<double>(0,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r02").as_double();
                                cameramatrix.at<double>(1,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r10").as_double();
                                cameramatrix.at<double>(1,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r11").as_double();
                                cameramatrix.at<double>(1,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r12").as_double();
                                cameramatrix.at<double>(2,0) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r20").as_double();
                                cameramatrix.at<double>(2,1) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r21").as_double();
                                cameramatrix.at<double>(2,2) = camnode.child("intrinsicparameters").child("cameramatrix").attribute("r22").as_double();

                                sysparameters.camintrinsicparameters[i].cameramatrix = cameramatrix.clone();

                                cameramatrixOther.at<double>(0,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r00").as_double();
                                cameramatrixOther.at<double>(0,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r01").as_double();
                                cameramatrixOther.at<double>(0,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r02").as_double();
                                cameramatrixOther.at<double>(1,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r10").as_double();
                                cameramatrixOther.at<double>(1,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r11").as_double();
                                cameramatrixOther.at<double>(1,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r12").as_double();
                                cameramatrixOther.at<double>(2,0) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r20").as_double();
                                cameramatrixOther.at<double>(2,1) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r21").as_double();
                                cameramatrixOther.at<double>(2,2) = camnode.child("intrinsicparameters").child("cameramatrixOther").attribute("r22").as_double();

                                sysparameters.camintrinsicparameters[i].cameramatrixOther = cameramatrixOther.clone();

                                R.at<double>(0,0) = camnode.child("extrinsicparameters").child("R").attribute("r00").as_double();
                                R.at<double>(0,1) = camnode.child("extrinsicparameters").child("R").attribute("r01").as_double();
                                R.at<double>(0,2) = camnode.child("extrinsicparameters").child("R").attribute("r02").as_double();
                                R.at<double>(1,0) = camnode.child("extrinsicparameters").child("R").attribute("r10").as_double();
                                R.at<double>(1,1) = camnode.child("extrinsicparameters").child("R").attribute("r11").as_double();
                                R.at<double>(1,2) = camnode.child("extrinsicparameters").child("R").attribute("r12").as_double();
                                R.at<double>(2,0) = camnode.child("extrinsicparameters").child("R").attribute("r20").as_double();
                                R.at<double>(2,1) = camnode.child("extrinsicparameters").child("R").attribute("r21").as_double();
                                R.at<double>(2,2) = camnode.child("extrinsicparameters").child("R").attribute("r22").as_double();
                                sysparameters.camextrinsicparameters[i].R = R.clone();

                                T.at<double>(0,0) = camnode.child("extrinsicparameters").child("T").attribute("r00").as_double();
                                T.at<double>(1,0) = camnode.child("extrinsicparameters").child("T").attribute("r01").as_double();
                                T.at<double>(2,0) = camnode.child("extrinsicparameters").child("T").attribute("r02").as_double();
                                sysparameters.camextrinsicparameters[i].T = T.clone();

                                E.at<double>(0,0) = camnode.child("extrinsicparameters").child("E").attribute("r00").as_double();
                                E.at<double>(0,1) = camnode.child("extrinsicparameters").child("E").attribute("r01").as_double();
                                E.at<double>(0,2) = camnode.child("extrinsicparameters").child("E").attribute("r02").as_double();
                                E.at<double>(1,0) = camnode.child("extrinsicparameters").child("E").attribute("r10").as_double();
                                E.at<double>(1,1) = camnode.child("extrinsicparameters").child("E").attribute("r11").as_double();
                                E.at<double>(1,2) = camnode.child("extrinsicparameters").child("E").attribute("r12").as_double();
                                E.at<double>(2,0) = camnode.child("extrinsicparameters").child("E").attribute("r20").as_double();
                                E.at<double>(2,1) = camnode.child("extrinsicparameters").child("E").attribute("r21").as_double();
                                E.at<double>(2,2) = camnode.child("extrinsicparameters").child("E").attribute("r22").as_double();
                                sysparameters.camextrinsicparameters[i].E = E.clone();

                                F.at<double>(0,0) = camnode.child("extrinsicparameters").child("F").attribute("r00").as_double();
                                F.at<double>(0,1) = camnode.child("extrinsicparameters").child("F").attribute("r01").as_double();
                                F.at<double>(0,2) = camnode.child("extrinsicparameters").child("F").attribute("r02").as_double();
                                F.at<double>(1,0) = camnode.child("extrinsicparameters").child("F").attribute("r10").as_double();
                                F.at<double>(1,1) = camnode.child("extrinsicparameters").child("F").attribute("r11").as_double();
                                F.at<double>(1,2) = camnode.child("extrinsicparameters").child("F").attribute("r12").as_double();
                                F.at<double>(2,0) = camnode.child("extrinsicparameters").child("F").attribute("r20").as_double();
                                F.at<double>(2,1) = camnode.child("extrinsicparameters").child("F").attribute("r21").as_double();
                                F.at<double>(2,2) = camnode.child("extrinsicparameters").child("F").attribute("r22").as_double();
                                sysparameters.camextrinsicparameters[i].F = F.clone();

                                R1.at<double>(0,0) = camnode.child("relativeparameters").child("R1").attribute("r00").as_double();
                                R1.at<double>(0,1) = camnode.child("relativeparameters").child("R1").attribute("r01").as_double();
                                R1.at<double>(0,2) = camnode.child("relativeparameters").child("R1").attribute("r02").as_double();
                                R1.at<double>(1,0) = camnode.child("relativeparameters").child("R1").attribute("r10").as_double();
                                R1.at<double>(1,1) = camnode.child("relativeparameters").child("R1").attribute("r11").as_double();
                                R1.at<double>(1,2) = camnode.child("relativeparameters").child("R1").attribute("r12").as_double();
                                R1.at<double>(2,0) = camnode.child("relativeparameters").child("R1").attribute("r20").as_double();
                                R1.at<double>(2,1) = camnode.child("relativeparameters").child("R1").attribute("r21").as_double();
                                R1.at<double>(2,2) = camnode.child("relativeparameters").child("R1").attribute("r22").as_double();
                                sysparameters.camrelativeparameters[i].R1 = R1.clone();

                                R2.at<double>(0,0) = camnode.child("relativeparameters").child("R2").attribute("r00").as_double();
                                R2.at<double>(0,1) = camnode.child("relativeparameters").child("R2").attribute("r01").as_double();
                                R2.at<double>(0,2) = camnode.child("relativeparameters").child("R2").attribute("r02").as_double();
                                R2.at<double>(1,0) = camnode.child("relativeparameters").child("R2").attribute("r10").as_double();
                                R2.at<double>(1,1) = camnode.child("relativeparameters").child("R2").attribute("r11").as_double();
                                R2.at<double>(1,2) = camnode.child("relativeparameters").child("R2").attribute("r12").as_double();
                                R2.at<double>(2,0) = camnode.child("relativeparameters").child("R2").attribute("r20").as_double();
                                R2.at<double>(2,1) = camnode.child("relativeparameters").child("R2").attribute("r21").as_double();
                                R2.at<double>(2,2) = camnode.child("relativeparameters").child("R2").attribute("r22").as_double();
                                sysparameters.camrelativeparameters[i].R2 = R2.clone();

                                P1.at<double>(0,0) = camnode.child("relativeparameters").child("P1").attribute("r00").as_double();
                                P1.at<double>(0,1) = camnode.child("relativeparameters").child("P1").attribute("r01").as_double();
                                P1.at<double>(0,2) = camnode.child("relativeparameters").child("P1").attribute("r02").as_double();
                                P1.at<double>(0,3) = camnode.child("relativeparameters").child("P1").attribute("r03").as_double();
                                P1.at<double>(1,0) = camnode.child("relativeparameters").child("P1").attribute("r10").as_double();
                                P1.at<double>(1,1) = camnode.child("relativeparameters").child("P1").attribute("r11").as_double();
                                P1.at<double>(1,2) = camnode.child("relativeparameters").child("P1").attribute("r12").as_double();
                                P1.at<double>(1,3) = camnode.child("relativeparameters").child("P1").attribute("r13").as_double();
                                P1.at<double>(2,0) = camnode.child("relativeparameters").child("P1").attribute("r20").as_double();
                                P1.at<double>(2,1) = camnode.child("relativeparameters").child("P1").attribute("r21").as_double();
                                P1.at<double>(2,2) = camnode.child("relativeparameters").child("P1").attribute("r22").as_double();
                                P1.at<double>(2,3) = camnode.child("relativeparameters").child("P1").attribute("r23").as_double();
                                sysparameters.camrelativeparameters[i].P1 = P1.clone();

                                P2.at<double>(0,0) = camnode.child("relativeparameters").child("P2").attribute("r00").as_double();
                                P2.at<double>(0,1) = camnode.child("relativeparameters").child("P2").attribute("r01").as_double();
                                P2.at<double>(0,2) = camnode.child("relativeparameters").child("P2").attribute("r02").as_double();
                                P2.at<double>(0,3) = camnode.child("relativeparameters").child("P2").attribute("r03").as_double();
                                P2.at<double>(1,0) = camnode.child("relativeparameters").child("P2").attribute("r10").as_double();
                                P2.at<double>(1,1) = camnode.child("relativeparameters").child("P2").attribute("r11").as_double();
                                P2.at<double>(1,2) = camnode.child("relativeparameters").child("P2").attribute("r12").as_double();
                                P2.at<double>(1,3) = camnode.child("relativeparameters").child("P2").attribute("r13").as_double();
                                P2.at<double>(2,0) = camnode.child("relativeparameters").child("P2").attribute("r20").as_double();
                                P2.at<double>(2,1) = camnode.child("relativeparameters").child("P2").attribute("r21").as_double();
                                P2.at<double>(2,2) = camnode.child("relativeparameters").child("P2").attribute("r22").as_double();
                                P2.at<double>(2,3) = camnode.child("relativeparameters").child("P2").attribute("r23").as_double();
                                sysparameters.camrelativeparameters[i].P2 = P2.clone();

                                Q.at<double>(0,0) = camnode.child("relativeparameters").child("Q").attribute("r00").as_double();
                                Q.at<double>(0,1) = camnode.child("relativeparameters").child("Q").attribute("r01").as_double();
                                Q.at<double>(0,2) = camnode.child("relativeparameters").child("Q").attribute("r02").as_double();
                                Q.at<double>(0,3) = camnode.child("relativeparameters").child("Q").attribute("r03").as_double();
                                Q.at<double>(1,0) = camnode.child("relativeparameters").child("Q").attribute("r10").as_double();
                                Q.at<double>(1,1) = camnode.child("relativeparameters").child("Q").attribute("r11").as_double();
                                Q.at<double>(1,2) = camnode.child("relativeparameters").child("Q").attribute("r12").as_double();
                                Q.at<double>(1,3) = camnode.child("relativeparameters").child("Q").attribute("r13").as_double();
                                Q.at<double>(2,0) = camnode.child("relativeparameters").child("Q").attribute("r20").as_double();
                                Q.at<double>(2,1) = camnode.child("relativeparameters").child("Q").attribute("r21").as_double();
                                Q.at<double>(2,2) = camnode.child("relativeparameters").child("Q").attribute("r22").as_double();
                                Q.at<double>(2,3) = camnode.child("relativeparameters").child("Q").attribute("r23").as_double();
                                Q.at<double>(3,0) = camnode.child("relativeparameters").child("Q").attribute("r30").as_double();
                                Q.at<double>(3,1) = camnode.child("relativeparameters").child("Q").attribute("r31").as_double();
                                Q.at<double>(3,2) = camnode.child("relativeparameters").child("Q").attribute("r32").as_double();
                                Q.at<double>(3,3) = camnode.child("relativeparameters").child("Q").attribute("r33").as_double();
                                sysparameters.camrelativeparameters[i].Q = Q.clone();

                                sysstatus.status = 3;
                                break;

                        }

                     }else{

                         std::cout << "IRMA////LOADCONFIG////Creating camera node with id " << _cameras[i]->getCameraSerial() <<" ,creating..." << std::endl;
                            pugi::xml_node newnode = root_node.append_child();
                            newnode.set_name("camera");
                                newnode.append_attribute("serialid");
                                newnode.attribute("serialid").set_value(_cameras[i]->getCameraSerial());
                                newnode.append_attribute("status");
                                newnode.attribute("status").set_value("0");

                            doccamconfig.save_file(sysfileconfig.configcampath);
                     }
                }
            }
        }
        return 0;

}

int IRMA::saveCamerasIntrinsicConfig(){
    pugi::xml_node root_node;
    root_node = doccamconfig.child("cameras");

        for(int i=0; i<sysbase.camsnum; i++){
            pugi::xml_node camnode;
            camnode= root_node.find_child_by_attribute("camera","serialid",_cameras[i]->getCameraSerial());
            camnode.attribute("status").set_value("1");
            camnode.remove_child("intrinsicparameters");
            camnode.append_child("intrinsicparameters");
                camnode.child("intrinsicparameters").append_child("distCoeffs");
                    camnode.child("intrinsicparameters").child("distCoeffs").append_attribute("k1")=sysparameters.camintrinsicparameters[i].distCoeffs.at<double>(0,0);
                    camnode.child("intrinsicparameters").child("distCoeffs").append_attribute("k2")=sysparameters.camintrinsicparameters[i].distCoeffs.at<double>(0,1);
                    camnode.child("intrinsicparameters").child("distCoeffs").append_attribute("p1")=sysparameters.camintrinsicparameters[i].distCoeffs.at<double>(0,2);
                    camnode.child("intrinsicparameters").child("distCoeffs").append_attribute("p2")=sysparameters.camintrinsicparameters[i].distCoeffs.at<double>(0,3);
                    if(sysparameters.camintrinsicparameters[i].distCoeffs.cols > 4)
                        camnode.child("intrinsicparameters").child("distCoeffs").append_attribute("k3")=sysparameters.camintrinsicparameters[i].distCoeffs.at<double>(0,4);
                    if(sysparameters.camintrinsicparameters[i].distCoeffs.cols > 5)
                        camnode.child("intrinsicparameters").child("distCoeffs").append_attribute("k4")=sysparameters.camintrinsicparameters[i].distCoeffs.at<double>(0,5);
                    if(sysparameters.camintrinsicparameters[i].distCoeffs.cols > 6)
                        camnode.child("intrinsicparameters").child("distCoeffs").append_attribute("k5")=sysparameters.camintrinsicparameters[i].distCoeffs.at<double>(0,6);
                    if(sysparameters.camintrinsicparameters[i].distCoeffs.cols > 7)
                        camnode.child("intrinsicparameters").child("distCoeffs").append_attribute("k6")=sysparameters.camintrinsicparameters[i].distCoeffs.at<double>(0,7);

                camnode.child("intrinsicparameters").append_child("cameramatrix");
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r00")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(0,0);
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r01")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(0,1);
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r02")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(0,2);
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r10")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(1,0);
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r11")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(1,1);
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r12")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(1,2);
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r20")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(2,0);
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r21")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(2,1);
                    camnode.child("intrinsicparameters").child("cameramatrix").append_attribute("r22")=sysparameters.camintrinsicparameters[i].cameramatrix.at<double>(2,2);

                camnode.child("intrinsicparameters").append_child("distCoeffsOther");
                    camnode.child("intrinsicparameters").child("distCoeffsOther").append_attribute("k1")=sysparameters.camintrinsicparameters[i].distCoeffsOther.at<double>(0,0);
                    camnode.child("intrinsicparameters").child("distCoeffsOther").append_attribute("k2")=sysparameters.camintrinsicparameters[i].distCoeffsOther.at<double>(0,1);
                    camnode.child("intrinsicparameters").child("distCoeffsOther").append_attribute("p1")=sysparameters.camintrinsicparameters[i].distCoeffsOther.at<double>(0,2);
                    camnode.child("intrinsicparameters").child("distCoeffsOther").append_attribute("p2")=sysparameters.camintrinsicparameters[i].distCoeffsOther.at<double>(0,3);
                    if(sysparameters.camintrinsicparameters[i].distCoeffsOther.cols > 4)
                        camnode.child("intrinsicparameters").child("distCoeffsOther").append_attribute("k3")=sysparameters.camintrinsicparameters[i].distCoeffsOther.at<double>(0,4);
                    if(sysparameters.camintrinsicparameters[i].distCoeffsOther.cols > 5)
                        camnode.child("intrinsicparameters").child("distCoeffsOther").append_attribute("k4")=sysparameters.camintrinsicparameters[i].distCoeffsOther.at<double>(0,5);
                    if(sysparameters.camintrinsicparameters[i].distCoeffsOther.cols > 6)
                        camnode.child("intrinsicparameters").child("distCoeffsOther").append_attribute("k5")=sysparameters.camintrinsicparameters[i].distCoeffsOther.at<double>(0,6);
                    if(sysparameters.camintrinsicparameters[i].distCoeffsOther.cols > 7)
                        camnode.child("intrinsicparameters").child("distCoeffsOther").append_attribute("k6")=sysparameters.camintrinsicparameters[i].distCoeffsOther.at<double>(0,7);

                camnode.child("intrinsicparameters").append_child("cameramatrixOther");
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r00")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(0,0);
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r01")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(0,1);
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r02")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(0,2);
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r10")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(1,0);
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r11")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(1,1);
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r12")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(1,2);
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r20")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(2,0);
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r21")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(2,1);
                    camnode.child("intrinsicparameters").child("cameramatrixOther").append_attribute("r22")=sysparameters.camintrinsicparameters[i].cameramatrixOther.at<double>(2,2);
    }
    doccamconfig.save_file(sysfileconfig.configcampath);
    return 0;
}

int IRMA::saveCamerasExtrinsicConfig(){
    pugi::xml_node root_node;
    root_node = doccamconfig.child("cameras");

        for(int i=0; i<sysbase.camsnum; i++){
            pugi::xml_node camnode;
            camnode= root_node.find_child_by_attribute("camera","serialid",_cameras[i]->getCameraSerial());
            camnode.attribute("status").set_value("2");
            camnode.remove_child("extrinsicparameters");
            camnode.append_child("extrinsicparameters");
                camnode.child("extrinsicparameters").append_child("R");
                    camnode.child("extrinsicparameters").child("R").append_attribute("r00")=sysparameters.camextrinsicparameters[i].R.at<double>(0,0);
                    camnode.child("extrinsicparameters").child("R").append_attribute("r01")=sysparameters.camextrinsicparameters[i].R.at<double>(0,1);
                    camnode.child("extrinsicparameters").child("R").append_attribute("r02")=sysparameters.camextrinsicparameters[i].R.at<double>(0,2);
                    camnode.child("extrinsicparameters").child("R").append_attribute("r10")=sysparameters.camextrinsicparameters[i].R.at<double>(1,0);
                    camnode.child("extrinsicparameters").child("R").append_attribute("r11")=sysparameters.camextrinsicparameters[i].R.at<double>(1,1);
                    camnode.child("extrinsicparameters").child("R").append_attribute("r12")=sysparameters.camextrinsicparameters[i].R.at<double>(1,2);
                    camnode.child("extrinsicparameters").child("R").append_attribute("r20")=sysparameters.camextrinsicparameters[i].R.at<double>(2,0);
                    camnode.child("extrinsicparameters").child("R").append_attribute("r21")=sysparameters.camextrinsicparameters[i].R.at<double>(2,1);
                    camnode.child("extrinsicparameters").child("R").append_attribute("r22")=sysparameters.camextrinsicparameters[i].R.at<double>(2,2);

                camnode.child("extrinsicparameters").append_child("T");
                    camnode.child("extrinsicparameters").child("T").append_attribute("r00")=sysparameters.camextrinsicparameters[i].T.at<double>(0,0);
                    camnode.child("extrinsicparameters").child("T").append_attribute("r01")=sysparameters.camextrinsicparameters[i].T.at<double>(1,0);
                    camnode.child("extrinsicparameters").child("T").append_attribute("r02")=sysparameters.camextrinsicparameters[i].T.at<double>(2,0);

                camnode.child("extrinsicparameters").append_child("E");
                    camnode.child("extrinsicparameters").child("E").append_attribute("r00")=sysparameters.camextrinsicparameters[i].E.at<double>(0,0);
                    camnode.child("extrinsicparameters").child("E").append_attribute("r01")=sysparameters.camextrinsicparameters[i].E.at<double>(0,1);
                    camnode.child("extrinsicparameters").child("E").append_attribute("r02")=sysparameters.camextrinsicparameters[i].E.at<double>(0,2);
                    camnode.child("extrinsicparameters").child("E").append_attribute("r10")=sysparameters.camextrinsicparameters[i].E.at<double>(1,0);
                    camnode.child("extrinsicparameters").child("E").append_attribute("r11")=sysparameters.camextrinsicparameters[i].E.at<double>(1,1);
                    camnode.child("extrinsicparameters").child("E").append_attribute("r12")=sysparameters.camextrinsicparameters[i].E.at<double>(1,2);
                    camnode.child("extrinsicparameters").child("E").append_attribute("r20")=sysparameters.camextrinsicparameters[i].E.at<double>(2,0);
                    camnode.child("extrinsicparameters").child("E").append_attribute("r21")=sysparameters.camextrinsicparameters[i].E.at<double>(2,1);
                    camnode.child("extrinsicparameters").child("E").append_attribute("r22")=sysparameters.camextrinsicparameters[i].E.at<double>(2,2);

                camnode.child("extrinsicparameters").append_child("F");
                    camnode.child("extrinsicparameters").child("F").append_attribute("r00")=sysparameters.camextrinsicparameters[i].F.at<double>(0,0);
                    camnode.child("extrinsicparameters").child("F").append_attribute("r01")=sysparameters.camextrinsicparameters[i].F.at<double>(0,1);
                    camnode.child("extrinsicparameters").child("F").append_attribute("r02")=sysparameters.camextrinsicparameters[i].F.at<double>(0,2);
                    camnode.child("extrinsicparameters").child("F").append_attribute("r10")=sysparameters.camextrinsicparameters[i].F.at<double>(1,0);
                    camnode.child("extrinsicparameters").child("F").append_attribute("r11")=sysparameters.camextrinsicparameters[i].F.at<double>(1,1);
                    camnode.child("extrinsicparameters").child("F").append_attribute("r12")=sysparameters.camextrinsicparameters[i].F.at<double>(1,2);
                    camnode.child("extrinsicparameters").child("F").append_attribute("r20")=sysparameters.camextrinsicparameters[i].F.at<double>(2,0);
                    camnode.child("extrinsicparameters").child("F").append_attribute("r21")=sysparameters.camextrinsicparameters[i].F.at<double>(2,1);
                    camnode.child("extrinsicparameters").child("F").append_attribute("r22")=sysparameters.camextrinsicparameters[i].F.at<double>(2,2);
        }
    doccamconfig.save_file(sysfileconfig.configcampath);
    return 0;
}

int IRMA::saveCamerasRelativeConfig(){
    pugi::xml_node root_node;
    root_node = doccamconfig.child("cameras");

        for(int i=0; i<sysbase.camsnum; i++){
            pugi::xml_node camnode;
            camnode= root_node.find_child_by_attribute("camera","serialid",_cameras[i]->getCameraSerial());
            camnode.attribute("status").set_value(3);
            camnode.remove_child("relativeparameters");
            camnode.append_child("relativeparameters");
                camnode.child("relativeparameters").append_child("R1");
                    camnode.child("relativeparameters").child("R1").append_attribute("r00")=sysparameters.camrelativeparameters[i].R1.at<double>(0,0);
                    camnode.child("relativeparameters").child("R1").append_attribute("r01")=sysparameters.camrelativeparameters[i].R1.at<double>(0,1);
                    camnode.child("relativeparameters").child("R1").append_attribute("r02")=sysparameters.camrelativeparameters[i].R1.at<double>(0,2);
                    camnode.child("relativeparameters").child("R1").append_attribute("r10")=sysparameters.camrelativeparameters[i].R1.at<double>(1,0);
                    camnode.child("relativeparameters").child("R1").append_attribute("r11")=sysparameters.camrelativeparameters[i].R1.at<double>(1,1);
                    camnode.child("relativeparameters").child("R1").append_attribute("r12")=sysparameters.camrelativeparameters[i].R1.at<double>(1,2);
                    camnode.child("relativeparameters").child("R1").append_attribute("r20")=sysparameters.camrelativeparameters[i].R1.at<double>(2,0);
                    camnode.child("relativeparameters").child("R1").append_attribute("r21")=sysparameters.camrelativeparameters[i].R1.at<double>(2,1);
                    camnode.child("relativeparameters").child("R1").append_attribute("r22")=sysparameters.camrelativeparameters[i].R1.at<double>(2,2);

                camnode.child("relativeparameters").append_child("R2");
                    camnode.child("relativeparameters").child("R2").append_attribute("r00")=sysparameters.camrelativeparameters[i].R2.at<double>(0,0);
                    camnode.child("relativeparameters").child("R2").append_attribute("r01")=sysparameters.camrelativeparameters[i].R2.at<double>(0,1);
                    camnode.child("relativeparameters").child("R2").append_attribute("r02")=sysparameters.camrelativeparameters[i].R2.at<double>(0,2);
                    camnode.child("relativeparameters").child("R2").append_attribute("r10")=sysparameters.camrelativeparameters[i].R2.at<double>(1,0);
                    camnode.child("relativeparameters").child("R2").append_attribute("r11")=sysparameters.camrelativeparameters[i].R2.at<double>(1,1);
                    camnode.child("relativeparameters").child("R2").append_attribute("r12")=sysparameters.camrelativeparameters[i].R2.at<double>(1,2);
                    camnode.child("relativeparameters").child("R2").append_attribute("r20")=sysparameters.camrelativeparameters[i].R2.at<double>(2,0);
                    camnode.child("relativeparameters").child("R2").append_attribute("r21")=sysparameters.camrelativeparameters[i].R2.at<double>(2,1);
                    camnode.child("relativeparameters").child("R2").append_attribute("r22")=sysparameters.camrelativeparameters[i].R2.at<double>(2,2);

                camnode.child("relativeparameters").append_child("P1");
                    camnode.child("relativeparameters").child("P1").append_attribute("r00")=sysparameters.camrelativeparameters[i].P1.at<double>(0,0);
                    camnode.child("relativeparameters").child("P1").append_attribute("r01")=sysparameters.camrelativeparameters[i].P1.at<double>(0,1);
                    camnode.child("relativeparameters").child("P1").append_attribute("r02")=sysparameters.camrelativeparameters[i].P1.at<double>(0,2);
                    camnode.child("relativeparameters").child("P1").append_attribute("r03")=sysparameters.camrelativeparameters[i].P1.at<double>(0,3);
                    camnode.child("relativeparameters").child("P1").append_attribute("r10")=sysparameters.camrelativeparameters[i].P1.at<double>(1,0);
                    camnode.child("relativeparameters").child("P1").append_attribute("r11")=sysparameters.camrelativeparameters[i].P1.at<double>(1,1);
                    camnode.child("relativeparameters").child("P1").append_attribute("r12")=sysparameters.camrelativeparameters[i].P1.at<double>(1,2);
                    camnode.child("relativeparameters").child("P1").append_attribute("r13")=sysparameters.camrelativeparameters[i].P1.at<double>(1,3);
                    camnode.child("relativeparameters").child("P1").append_attribute("r20")=sysparameters.camrelativeparameters[i].P1.at<double>(2,0);
                    camnode.child("relativeparameters").child("P1").append_attribute("r21")=sysparameters.camrelativeparameters[i].P1.at<double>(2,1);
                    camnode.child("relativeparameters").child("P1").append_attribute("r22")=sysparameters.camrelativeparameters[i].P1.at<double>(2,2);
                    camnode.child("relativeparameters").child("P1").append_attribute("r23")=sysparameters.camrelativeparameters[i].P1.at<double>(2,3);

                camnode.child("relativeparameters").append_child("P2");
                    camnode.child("relativeparameters").child("P2").append_attribute("r00")=sysparameters.camrelativeparameters[i].P2.at<double>(0,0);
                    camnode.child("relativeparameters").child("P2").append_attribute("r01")=sysparameters.camrelativeparameters[i].P2.at<double>(0,1);
                    camnode.child("relativeparameters").child("P2").append_attribute("r02")=sysparameters.camrelativeparameters[i].P2.at<double>(0,2);
                    camnode.child("relativeparameters").child("P2").append_attribute("r03")=sysparameters.camrelativeparameters[i].P2.at<double>(0,3);
                    camnode.child("relativeparameters").child("P2").append_attribute("r10")=sysparameters.camrelativeparameters[i].P2.at<double>(1,0);
                    camnode.child("relativeparameters").child("P2").append_attribute("r11")=sysparameters.camrelativeparameters[i].P2.at<double>(1,1);
                    camnode.child("relativeparameters").child("P2").append_attribute("r12")=sysparameters.camrelativeparameters[i].P2.at<double>(1,2);
                    camnode.child("relativeparameters").child("P2").append_attribute("r13")=sysparameters.camrelativeparameters[i].P2.at<double>(1,3);
                    camnode.child("relativeparameters").child("P2").append_attribute("r20")=sysparameters.camrelativeparameters[i].P2.at<double>(2,0);
                    camnode.child("relativeparameters").child("P2").append_attribute("r21")=sysparameters.camrelativeparameters[i].P2.at<double>(2,1);
                    camnode.child("relativeparameters").child("P2").append_attribute("r22")=sysparameters.camrelativeparameters[i].P2.at<double>(2,2);
                    camnode.child("relativeparameters").child("P2").append_attribute("r23")=sysparameters.camrelativeparameters[i].P2.at<double>(2,3);

                camnode.child("relativeparameters").append_child("Q");
                    camnode.child("relativeparameters").child("Q").append_attribute("r00")=sysparameters.camrelativeparameters[i].Q.at<double>(0,0);
                    camnode.child("relativeparameters").child("Q").append_attribute("r01")=sysparameters.camrelativeparameters[i].Q.at<double>(0,1);
                    camnode.child("relativeparameters").child("Q").append_attribute("r02")=sysparameters.camrelativeparameters[i].Q.at<double>(0,2);
                    camnode.child("relativeparameters").child("Q").append_attribute("r03")=sysparameters.camrelativeparameters[i].Q.at<double>(0,3);
                    camnode.child("relativeparameters").child("Q").append_attribute("r10")=sysparameters.camrelativeparameters[i].Q.at<double>(1,0);
                    camnode.child("relativeparameters").child("Q").append_attribute("r11")=sysparameters.camrelativeparameters[i].Q.at<double>(1,1);
                    camnode.child("relativeparameters").child("Q").append_attribute("r12")=sysparameters.camrelativeparameters[i].Q.at<double>(1,2);
                    camnode.child("relativeparameters").child("Q").append_attribute("r13")=sysparameters.camrelativeparameters[i].Q.at<double>(1,3);
                    camnode.child("relativeparameters").child("Q").append_attribute("r20")=sysparameters.camrelativeparameters[i].Q.at<double>(2,0);
                    camnode.child("relativeparameters").child("Q").append_attribute("r21")=sysparameters.camrelativeparameters[i].Q.at<double>(2,1);
                    camnode.child("relativeparameters").child("Q").append_attribute("r22")=sysparameters.camrelativeparameters[i].Q.at<double>(2,2);
                    camnode.child("relativeparameters").child("Q").append_attribute("r23")=sysparameters.camrelativeparameters[i].Q.at<double>(2,3);
                    camnode.child("relativeparameters").child("Q").append_attribute("r30")=sysparameters.camrelativeparameters[i].Q.at<double>(3,0);
                    camnode.child("relativeparameters").child("Q").append_attribute("r31")=sysparameters.camrelativeparameters[i].Q.at<double>(3,1);
                    camnode.child("relativeparameters").child("Q").append_attribute("r32")=sysparameters.camrelativeparameters[i].Q.at<double>(3,2);
                    camnode.child("relativeparameters").child("Q").append_attribute("r33")=sysparameters.camrelativeparameters[i].Q.at<double>(3,3);

        }
    doccamconfig.save_file(sysfileconfig.configcampath);
    return 0;
}


int IRMA::loadDetectorConfig(){

    if (!docmarkerconfig.load_file(sysfileconfig.configmarkerpath))
	{
		std::cerr << "IRMA////LOADCONFIG////Document configmarkers.xml does not exist creating a new one..." << std::endl;
        //return -1;
        docmarkerconfig.save_file(sysfileconfig.configmarkerpath);
        this->loadDetectorConfig(); //recursive -> go to the else
	}else{
        pugi::xml_node root_node;
        if (!(root_node = docmarkerconfig.child("blobdetector")))
        {
            std::cerr << "IRMA////LOADCONFIG////Document has not a valid config. remaking xml nodes..." << std::endl;
            docmarkerconfig.append_child("blobdetector");
                docmarkerconfig.child("blobdetector").append_attribute("minThreshold")= "40";
                docmarkerconfig.child("blobdetector").append_attribute("maxThreshold")= "60";
                docmarkerconfig.child("blobdetector").append_attribute("thresholdStep")= "5";
                docmarkerconfig.child("blobdetector").append_attribute("minArea")= "10";
                docmarkerconfig.child("blobdetector").append_attribute("maxArea")= "8000";
                docmarkerconfig.child("blobdetector").append_attribute("minConvexity")= ".4f";
                docmarkerconfig.child("blobdetector").append_attribute("maxConvexity")= "2";
                docmarkerconfig.child("blobdetector").append_attribute("minInertiaRatio")= ".1f";
                docmarkerconfig.child("blobdetector").append_attribute("filterByColor")= "0";
                docmarkerconfig.child("blobdetector").append_attribute("filterByCircularity")= "0";

            docmarkerconfig.save_file(sysfileconfig.configmarkerpath);
            //return -2;
            this->loadDetectorConfig();
        }else{
                sysblobdetectorparameters.minThreshold = root_node.attribute("minThreshold").as_int();
                sysblobdetectorparameters.maxThreshold = root_node.attribute("maxThreshold").as_int();
                sysblobdetectorparameters.thresholdStep = root_node.attribute("thresholdStep").as_float();
                sysblobdetectorparameters.minArea = root_node.attribute("minArea").as_float();
                sysblobdetectorparameters.maxArea = root_node.attribute("maxArea").as_float();
                sysblobdetectorparameters.minConvexity = root_node.attribute("minConvexity").as_float();
                sysblobdetectorparameters.maxConvexity = root_node.attribute("maxConvexity").as_float();
                sysblobdetectorparameters.minInertiaRatio = root_node.attribute("minInertiaRatio").as_float();
                sysblobdetectorparameters.filterByColor = root_node.attribute("filterByColor").as_bool();
                sysblobdetectorparameters.filterByCircularity = root_node.attribute("filterByCircularity").as_bool();

                bdetector = cv::SimpleBlobDetector(sysblobdetectorparameters);
                bdetector.create("SimpleBlobDetector");

        }
        if (!(root_node = docmarkerconfig.child("markerdetector")))
        {
            std::cerr << "IRMA////LOADCONFIG////Document has not a valid config. remaking xml nodes..." << std::endl;
            docmarkerconfig.append_child("markerdetector");
                docmarkerconfig.child("markerdetector").append_attribute("minThreshold")= "10";
                docmarkerconfig.child("markerdetector").append_attribute("maxThreshold")= "255";
                docmarkerconfig.child("markerdetector").append_attribute("mandatoryangles")= "false";
                docmarkerconfig.child("markerdetector").append_attribute("mandatoryangles")= "false";

                docmarkerconfig.save_file(sysfileconfig.configmarkerpath);
                //return -2;
                this->loadDetectorConfig();
        }else{

            sysmarkerparameters.minThreshold = root_node.attribute("minThreshold").as_int();
            sysmarkerparameters.maxThreshold = root_node.attribute("maxThreshold").as_int();
            sysmarkerparameters.aperturesize = 5;
            sysmarkerparameters.kerneldims = 3;
            sysmarkerparameters.mandatoryangles = root_node.attribute("mandatoryangles").as_bool();
            sysmarkerparameters.mandatoryangles = root_node.attribute("restablishpositions").as_bool();

        }
         return 0;
	}
}


int IRMA::loadPatternsConfig(){
        if (!docmarkerconfig.load_file(sysfileconfig.configmarkerpath))
        {
            std::cerr << "IRMA////LOADPATTERNSCONFIG////Document configmarkers.xml does not exist, creating a new one..." << std::endl;
            docmarkerconfig.save_file(sysfileconfig.configmarkerpath);
            this->loadPatternsConfig();
            //return -1;
        }else{
            pugi::xml_node root_node;
            if(!(root_node = docmarkerconfig.child("patterns"))){
                std::cout << "IRMA////LOADCONFIG////There's no patterns tag, remaking nodes..." << std::endl;
                docmarkerconfig.append_child("patterns");
                docmarkerconfig.save_file(sysfileconfig.configmarkerpath);
                //return -2
                this->loadPatternsConfig();

            }else{
                for(int i=0; i<sysbase.patnum; i++){
                     pugi::xml_node patternnode;

                     char patid[2];
                     sprintf(patid,"%d",i);
                     if(patternnode= root_node.find_child_by_attribute("pattern","id",patid)){

                                syspatterns.patterns[i].id = patternnode.attribute("id").as_int();
                                syspatterns.patterns[i].vertexnum = patternnode.attribute("vertexnum").as_int();
                                double minangle = patternnode.attribute("minangle").as_double();
                                double minanglecos = cos ( minangle * PI / 180.0 );
                                double maxangle = patternnode.attribute("maxangle").as_double();
                                double maxanglecos = cos ( maxangle * PI / 180.0 );
                                if(minanglecos < maxanglecos){
                                    syspatterns.patterns[i].mincos = minanglecos;
                                    syspatterns.patterns[i].maxcos = maxanglecos;
                                }else{
                                    syspatterns.patterns[i].maxcos = minanglecos;
                                    syspatterns.patterns[i].mincos = maxanglecos;
                                }

                     }else{

                            pugi::xml_node newnode = root_node.append_child();
                            newnode.set_name("pattern");
                                newnode.append_attribute("id");
                                newnode.attribute("id").set_value(i);
                                newnode.append_attribute("vertexnum");
                                newnode.attribute("vertexnum").set_value(3+i);
                                newnode.append_attribute("minangle");
                                newnode.attribute("minangle").set_value((3+i)*18);
                                newnode.append_attribute("maxangle");
                                newnode.attribute("maxangle").set_value((3+i)*24);

                            docmarkerconfig.save_file(sysfileconfig.configmarkerpath);
                            //return -3
                            i--;
                     }
                }
            }
        }
        return 0;
}

int IRMA::loadOriginConfig(){
    if (!docmarkerconfig.load_file(sysfileconfig.configmarkerpath))
    {
        std::cerr << "IRMA////LOADPATTERNSCONFIG////Document configmarkers.xml does not exist, creating a new one..." << std::endl;
        docmarkerconfig.save_file(sysfileconfig.configmarkerpath);
        this->loadOriginConfig();
        //return -1;
    }else{
        pugi::xml_node root_node;
        if(!(root_node = docmarkerconfig.child("origin"))){
            std::cout << "IRMA////LOADCONFIG////There's no origin tag, remaking nodes..." << std::endl;
            docmarkerconfig.append_child("origin");
                docmarkerconfig.child("origin").append_attribute("found");
                docmarkerconfig.child("origin").attribute("found").set_value(false);
                docmarkerconfig.child("origin").append_attribute("x");
                docmarkerconfig.child("origin").attribute("x").set_value(0);
                docmarkerconfig.child("origin").append_attribute("y");
                docmarkerconfig.child("origin").attribute("y").set_value(0);
                docmarkerconfig.child("origin").append_attribute("z");
                docmarkerconfig.child("origin").attribute("z").set_value(0);

            docmarkerconfig.save_file(sysfileconfig.configmarkerpath);
            //return -2
            this->loadOriginConfig();

        }else{
                if(root_node.attribute("found").as_bool() == false)
                        _origin->infoundcams = 0;
                else
                        _origin->infoundcams = -1;

                _origin->_eucl3d_ps.at<double>(0) = root_node.attribute("x").as_double();
                _origin->_eucl3d_ps.at<double>(1) = root_node.attribute("y").as_double();
                _origin->_eucl3d_ps.at<double>(2) = root_node.attribute("z").as_double();

            }
        }

    return 0;

}
int IRMA::saveOriginConfig(){

    pugi::xml_node root_node;
    root_node = docmarkerconfig.child("origin");
    root_node.attribute("found").set_value(true);

    root_node.attribute("x").set_value(_origin->_eucl3d_ps.at<double>(0));
    root_node.attribute("y").set_value(_origin->_eucl3d_ps.at<double>(1));
    root_node.attribute("z").set_value(_origin->_eucl3d_ps.at<double>(2));

    docmarkerconfig.save_file(sysfileconfig.configmarkerpath);

    return 0;
}

