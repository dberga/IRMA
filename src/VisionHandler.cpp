#include "IRMA.hpp"


void IRMA::CalibCaptureBoard(){


    if(sysmomentum._frames_board_captured.size() < sysboard.ncaptures){
    std::vector<cv::Point3f> objectCorners;
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Mat> currentcaptures;

        for(int i=0; i< _cameras.size(); i++){
                bool found = false;
                cv::Size patternsize(sysboard.width, sysboard.height);
                //get imagecorners
                found=cv::findChessboardCorners(sysmomentum._frames[i], patternsize, imageCorners); //imageconrers = centers?
                //get objectcorners //is permanently the same, the board conf
                for( int x = 0; x < sysboard.height; x++ )
                 for( int y = 0; y < sysboard.width; y++ )
                    objectCorners.push_back(cv::Point3f(x, y, sysboard.entitysize));



                //save imagecorners & objectcorners
                if(imageCorners.size() == patternsize.area()){
                    sysparameters.camcalibrateparameters[i].objectPoints.push_back(objectCorners); //objectcorners done
                    sysparameters.camcalibrateparameters[i].imagePoints.push_back(imageCorners);
                    currentcaptures.push_back(sysmomentum._frames[i].clone());

                        //cv::Mat drawncorners = sysmomentum._frames[i].clone();
                        //cv::drawChessboardCorners(drawncorners, patternsize, imageCorners, found);
                        //cv::imwrite( "chess.png",drawncorners );
                    std::cout << "CAM["<<i<<"]////boardpoints taken #"<< sysparameters.camcalibrateparameters[i].imagePoints.size() <<  std::endl;
                    imageCorners.clear();
                    objectCorners.clear();
                    found = false;
                }else{
                    std::cout << "CAM["<<i<<"]////boardpoints not taken #"<< sysparameters.camcalibrateparameters[i].imagePoints.size() <<  std::endl;
                }
        }

        if(currentcaptures.size() > 0){
            sysmomentum._frames_board_captured.push_back(currentcaptures);
        }

    }else{

                std::cout << "IRMA////CAMERAS ALREADY CALIBRATED"<< std::endl;


    }
}

void IRMA::CalibEraseBadCaptures(){

    int captured = 9999;
    for(int i=0; i< _cameras.size(); i++){
        if(sysparameters.camcalibrateparameters[i].imagePoints.size() <= captured)
            captured = sysparameters.camcalibrateparameters[i].imagePoints.size();
    }
    //captured is the min value of good captures

    for(int i=0; i< _cameras.size(); i++){
        if(sysparameters.camcalibrateparameters[i].imagePoints.size() > captured){
            sysparameters.camcalibrateparameters[i].imagePoints.pop_back();
            sysparameters.camcalibrateparameters[i].objectPoints.pop_back();
            sysmomentum._frames_board_captured.pop_back();
        }
    }
    //if the number of captures done by the cam "i" is bigger than the min value of good captures, pop back the last capture

        //change status if all good
        for(int i=0; i< _cameras.size(); i++){
            if(sysmomentum._frames_board_captured.size() >= sysboard.ncaptures){
                        sysstatus.camerastatus[i] = 1;
            }
        }
}

void IRMA::StereoCalibrateCameras(int camreference){

    cv::Size basesize(sysbase.width,sysbase.height);

    for(int i=0; i< _cameras.size(); i++){



        cv::stereoCalibrate(sysparameters.camcalibrateparameters[i].objectPoints,
                            sysparameters.camcalibrateparameters[camreference].imagePoints,
                            sysparameters.camcalibrateparameters[i].imagePoints,
                            sysparameters.camintrinsicparameters[i].cameramatrixOther,
                            sysparameters.camintrinsicparameters[i].distCoeffsOther,
                            sysparameters.camintrinsicparameters[i].cameramatrix,
                            sysparameters.camintrinsicparameters[i].distCoeffs,
                            basesize,
                            sysparameters.camextrinsicparameters[i].R,
                            sysparameters.camextrinsicparameters[i].T,
                            sysparameters.camextrinsicparameters[i].E,
                            sysparameters.camextrinsicparameters[i].F,
                            /*
                            cv::TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),
                            //CV_CALIB_FIX_ASPECT_RATIO +
                            //CV_CALIB_ZERO_TANGENT_DIST +
                            //CV_CALIB_SAME_FOCAL_LENGTH +
                            CV_CALIB_RATIONAL_MODEL
                            //CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5
                            */
                            cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                            CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST
                            );

    }

}


void IRMA::StereoRectifyCameras(int camreference){

    cv::Size basesize(sysbase.width,sysbase.height);
    for(int i=0; i< _cameras.size(); i++){


        cv::stereoRectify(sysparameters.camintrinsicparameters[i].cameramatrixOther,
                          sysparameters.camintrinsicparameters[i].distCoeffsOther,
                          sysparameters.camintrinsicparameters[i].cameramatrix,
                          sysparameters.camintrinsicparameters[i].distCoeffs,
                          basesize,
                          sysparameters.camextrinsicparameters[i].R,
                          sysparameters.camextrinsicparameters[i].T,
                          sysparameters.camrelativeparameters[i].R1,
                          sysparameters.camrelativeparameters[i].R2,
                          sysparameters.camrelativeparameters[i].P1,
                          sysparameters.camrelativeparameters[i].P2,
                          sysparameters.camrelativeparameters[i].Q
                          );
    }
}

void IRMA::TriangulateMarkerPoints(int camreference, int marker){


    for(int i=0; i< _cameras.size(); i++){
             cv::Mat puntc1(2,1,CV_64F);
              cv::Mat puntc2(2,1,CV_64F);
                puntc1.at<double>(0,0) = syspoints._markers_pos_pt[camreference][marker].x;
                puntc1.at<double>(1,0) = syspoints._markers_pos_pt[camreference][marker].y;

                puntc2.at<double>(0,0) = syspoints._markers_pos_pt[i][marker].x;
                puntc2.at<double>(1,0) = syspoints._markers_pos_pt[i][marker].y;

            cv::triangulatePoints(sysparameters.camrelativeparameters[i].P1,
                                  sysparameters.camrelativeparameters[i].P2,
                                  puntc1,
                                  puntc2,
                                  _markers[marker]->_homo3d_ps
                                  );

    }


}

void IRMA::TriangulateOrigin(int camreference){
    for(int i=0; i< _cameras.size(); i++){
            cv::Mat puntc1(2,1,CV_64F);
              cv::Mat puntc2(2,1,CV_64F);

                puntc1.at<double>(0,0) = syspoints._origin_pos_pt[camreference].x;
                puntc1.at<double>(1,0) = syspoints._origin_pos_pt[camreference].y;

                puntc2.at<double>(0,0) = syspoints._origin_pos_pt[i].x;
                puntc2.at<double>(1,0) = syspoints._origin_pos_pt[i].y;

            cv::triangulatePoints(sysparameters.camrelativeparameters[i].P1,
                                  sysparameters.camrelativeparameters[i].P2,
                                  puntc1,
                                  puntc2,
                                  _origin->_homo3d_ps
                                  );
    }
}

void IRMA::setMarkersHomoPointToEuclideanPoint(int marker){



             _markers[marker]->_eucl3d_ps.at<double>(0,0) = _markers[marker]->_homo3d_ps.at<double>(0,0) / _markers[marker]->_homo3d_ps.at<double>(0,3);
            _markers[marker]->_eucl3d_ps.at<double>(0,1) = _markers[marker]->_homo3d_ps.at<double>(0,1) / _markers[marker]->_homo3d_ps.at<double>(0,3);
            _markers[marker]->_eucl3d_ps.at<double>(0,2) = _markers[marker]->_homo3d_ps.at<double>(0,2) / _markers[marker]->_homo3d_ps.at<double>(0,3);



}

void IRMA::setOriginHomoPointToEuclideanPoint(){


            _origin->_eucl3d_ps.at<double>(0,0) = _origin->_homo3d_ps.at<double>(0,0) / _origin->_homo3d_ps.at<double>(0,3);
            _origin->_eucl3d_ps.at<double>(0,1) = _origin->_homo3d_ps.at<double>(0,1) / _origin->_homo3d_ps.at<double>(0,3);
            _origin->_eucl3d_ps.at<double>(0,2) = _origin->_homo3d_ps.at<double>(0,2) / _origin->_homo3d_ps.at<double>(0,3);

}

void IRMA::set3DWorld(int marker){


        _markers[marker]->_eucl3d_ps.at<double>(0,0) -=_origin->_eucl3d_ps.at<double>(0,0);
        _markers[marker]->_eucl3d_ps.at<double>(0,1) -=_origin->_eucl3d_ps.at<double>(0,1);
        _markers[marker]->_eucl3d_ps.at<double>(0,2) -=_origin->_eucl3d_ps.at<double>(0,2);



}
