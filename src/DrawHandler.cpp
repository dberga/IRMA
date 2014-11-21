#include "IRMA.hpp"


    void IRMA::getFrames(){
        for(int i=0; i< _cameras.size(); i++){
            sysmomentum._frames[i] = _cameras[i]->getFrame();
        }
    }

    void IRMA::drawFrames(){
        for(int i=0; i< _cameras.size(); i++){
            sysmomentum._frames_show[i] = sysmomentum._frames[i].clone();
        }
    }
    void IRMA::getFramesUndistorted(){
        for(int i=0; i< _cameras.size(); i++){
            sysmomentum._frames[i] = _cameras[i]->getFrame();
            sysmomentum._frames_undistorted[i] = sysmomentum._frames[i].clone();
            cv::undistort(sysmomentum._frames[i],
                          sysmomentum._frames_undistorted[i],
                          sysparameters.camintrinsicparameters[i].cameramatrix,
                          sysparameters.camintrinsicparameters[i].distCoeffs
                          );
        }
    }
    void IRMA::drawFramesUndistorted(){
        for(int i=0; i< _cameras.size(); i++){
            sysmomentum._frames_show[i] = sysmomentum._frames_undistorted[i].clone();
        }
    }
std::string IRMA::double_a_string(double x)
{
  std::ostringstream ss;
  ss << x;
  return ss.str();
}
    void IRMA::drawBlobs(){


        for(int i=0; i< _cameras.size(); i++){
            sysmomentum._frames_blobbed[i] = sysmomentum._frames_show[i].clone();

            cv::drawKeypoints(sysmomentum._frames_show[i],
                              syspoints._blobs[i],
                              sysmomentum._frames_blobbed[i],
                              CV_RGB(255,0,0), 4);


        }

        for(int i=0; i< _cameras.size(); i++)
            sysmomentum._frames_show[i] = sysmomentum._frames_blobbed[i].clone();
    }

    void IRMA::drawMarkers(){

        for(int i=0; i< _cameras.size(); i++){
            cv::Mat accummarkers = sysmomentum._frames_show[i].clone();
            for(int h=0; h< shapes[i].size(); h++){
                for(int c=0; c< shapes[i][h].contours.size(); c++){

                    if(c+1 < shapes[i][h].contours.size())
                        cv::line(accummarkers,shapes[i][h].contours[c],shapes[i][h].contours[c+1],CV_RGB(0,255,0));

                }
            }
            sysmomentum._frames_marked[i] = accummarkers.clone();
        }



        for(int i=0; i< _cameras.size(); i++){
            cv::Mat accummarkers = sysmomentum._frames_marked[i].clone();

            for(int j=0; j< syspoints._markers_pos_pt[i].size(); j++){


                char markername[1] = "";
                sprintf(markername, "%d", j);

                cv::putText(accummarkers,
                            markername,
                            syspoints._markers_pos_pt[i][j],
                            cv::FONT_HERSHEY_SIMPLEX,
                            1,
                            cv::Scalar(0,200,0),
                            2,
                            16);

            }
            sysmomentum._frames_marked[i] = accummarkers.clone();
        }

        for(int i=0; i< _cameras.size(); i++)
            sysmomentum._frames_show[i] = sysmomentum._frames_marked[i].clone();
    }



void IRMA::drawCanny(){
    for(int i=0; i< _cameras.size(); i++)
            sysmomentum._frames_show[i] = sysmomentum._frames_binary[i].clone();
}
void IRMA::drawOrigin(){
    if(_origin->infoundcams == -1){
        for(int i=0; i< _cameras.size(); i++){
                cv::Mat circlmat = sysmomentum._frames_show[i].clone();

                       cv::circle(  circlmat,
                                    syspoints._origin_pos_pt[i],
                                    10,
                                    CV_RGB(0,0,255)
                               );
                sysmomentum._frames_show[i] = circlmat.clone();
        }
    }

}
void IRMA::drawMarkers3D(){



    for(int i=0; i< _cameras.size(); i++){
            cv::Mat accummarkers3d = sysmomentum._frames_show[i].clone();

            for(int m=0; m < _markers.size(); m++){

            for(int h=0; h< shapes[i].size(); h++){


                if(shapes[i][h].type == _markers[m]->pat.id ){ //&& _markers[m]->infoundcams > 1

                        char markereuclpos[23] = "";


                        std::string mX = double_a_string(_markers[m]->_eucl3d_ps.at<double>(0));
                        std::string mY = double_a_string(_markers[m]->_eucl3d_ps.at<double>(1));
                        std::string mZ = double_a_string(_markers[m]->_eucl3d_ps.at<double>(2));


                        std::strcat(markereuclpos,mX.c_str());
                        std::strcat(markereuclpos,",");
                        std::strcat(markereuclpos,mY.c_str());
                        std::strcat(markereuclpos,",");
                        std::strcat(markereuclpos,mZ.c_str());


                        cv::putText(accummarkers3d,
                                    markereuclpos,
                                    shapes[i][h].contours[shapes[i][h].contours.size()-1],
                                    cv::FONT_HERSHEY_SIMPLEX,
                                    0.50,
                                    cv::Scalar(0,200,0),
                                    1,
                                    10);
                    }
                }
            }
            sysmomentum._frames_show[i] = accummarkers3d.clone();
        }
/*
    for(int i=0; i< _markers.size(); i++){
        std::cout << "3DMARKER ["<< i << "]="<< _markers[i]->_eucl3d_ps << std::endl;
    }
    std::cout << std::endl;
*/


}

void IRMA::draw(){

    drawFrames();

    switch (sysstatus.status){
        case -1: //no cameras connected
            std::cout << "IRMA////DRAW// NO CAMERAS CONNECTED" << std::endl;
            break;
        case 0: //cameras connected
            if(sysmomentum.drawblobs)
                drawBlobs();
            if(sysmomentum.drawmarkers)
                drawMarkers();
            if(sysmomentum.drawcanny)
                drawCanny();
            break;
        case 1: //cameras calibrated
            drawFramesUndistorted();
            if(sysmomentum.drawcanny)
                drawCanny();
            if(sysmomentum.drawblobs)
                drawBlobs();
            if(sysmomentum.drawmarkers)
                drawMarkers();
            break;
        case 2: //cameras stereocalibrated
            drawFramesUndistorted();
            if(sysmomentum.drawcanny)
                drawCanny();
            if(sysmomentum.drawblobs)
                drawBlobs();
            if(sysmomentum.drawmarkers)
                drawMarkers();
            break;
        case 3: //cameras rectified
            drawFramesUndistorted();
            preProcess();
            if(sysmomentum.drawcanny)
                drawCanny();
            if(sysmomentum.drawblobs)
                drawBlobs();
            if(sysmomentum.drawmarkers)
                drawMarkers();
            if(sysmomentum.drawmarkers3D){
                drawOrigin();
                drawMarkers3D();
            }

            break;
    }
}

void IRMA::iterate(){



    switch (sysstatus.status){
        case -1: //no cameras connected
            std::cout << "IRMA////DRAW// NO CAMERAS CONNECTED" << std::endl;
            break;
        case 0: //cameras connected
            getFrames();
            preProcess();
            if(sysmomentum.drawblobs)
                detectBlobs();
            if(sysmomentum.drawmarkers)
                detectMarkers();
            break;
        case 1: //cameras calibrated
            getFramesUndistorted();
            preProcess();
            if(sysmomentum.drawblobs)
                detectBlobs();
            if(sysmomentum.drawmarkers)
                detectMarkers();
            break;
        case 2: //cameras stereocalibrated
            getFramesUndistorted();
            preProcess();
            if(sysmomentum.drawblobs)
                detectBlobs();
            if(sysmomentum.drawmarkers)
                detectMarkers();

            break;
        case 3: //cameras rectified
            getFramesUndistorted();
            preProcess();
            if(sysmomentum.drawblobs)
                detectBlobs();
            if(sysmomentum.drawmarkers)
                detectMarkers();
            if(sysmomentum.drawmarkers3D){
                detectMarkers3D();
            }
            break;
    }
}
