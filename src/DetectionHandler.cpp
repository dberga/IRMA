#include "IRMA.hpp"


void IRMA::detectBlobs(){ //keep in mind of pushing back and clearing when getting news

    for(int i=0; i<_cameras.size(); i++){
        syspoints._blobs[i].clear();
        bdetector.detect(sysmomentum._frames[i], syspoints._blobs[i]);
    }
}

void IRMA::detectMarkers(){

    getShapes();
    getMarkers();
}

void IRMA::detectOrigin(){
    if(_origin->infoundcams > -1){
            getOrigin();
            //std::cout << "originfound in "<<_origin->infoundcams << std::endl;
            if(_origin->infoundcams >= 2 && syspoints.originfoundinbasecam==true){

                TriangulateOrigin(sysbase.basecam);
                setOriginHomoPointToEuclideanPoint();

                saveOriginConfig();
                std::cout << "origin captured" << std::endl;

                _origin->infoundcams = -1;
            }else{
                _origin->infoundcams = 0;
                syspoints.originfoundinbasecam = false;
            }

    }else{
            getOrigin();
            //std::cout << "originfound in "<<_origin->infoundcams << std::endl;
            if(_origin->infoundcams >= 2 && syspoints.originfoundinbasecam==true){

                TriangulateOrigin(0);
                setOriginHomoPointToEuclideanPoint();

                saveOriginConfig();
                std::cout << "origin captured" << std::endl;

                _origin->infoundcams = -1;
            }else{

            }
        //already triangulated (found 3d point) no need to find the origin again
    }

}

void IRMA::preProcess(){
    for(int i=0; i<_cameras.size(); i++){

        cv::Mat blurred;
        if(sysstatus.status>0)
            blurred = sysmomentum._frames_undistorted[i].clone();
        else
            blurred = sysmomentum._frames[i].clone();

       cv::cvtColor(blurred, blurred, CV_BGR2GRAY);


        if(sysmarkerparameters.aperturesize %2 == 0)
            sysmarkerparameters.aperturesize++;

        if(sysmarkerparameters.aperturesize <= 1)
            sysmarkerparameters.aperturesize = 3;

        if(sysmarkerparameters.kerneldims %2 == 0)
            sysmarkerparameters.kerneldims++;

        if(sysmarkerparameters.kerneldims <= 1)
            sysmarkerparameters.kerneldims = 1;

      cv::blur( blurred, blurred, Size(sysmarkerparameters.kerneldims, sysmarkerparameters.kerneldims) );

       cv::Canny(blurred, sysmomentum._frames_binary[i], sysmarkerparameters.minThreshold, sysmarkerparameters.maxThreshold,sysmarkerparameters.aperturesize);
    }

}

double IRMA::getCosine(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void IRMA::getShapes(){




    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Point> approx;

    for(int i=0; i<_cameras.size(); i++){
        for(int j=0;j<_markers.size();j++){
        cv::findContours(sysmomentum._frames_binary[i], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


        for(int x=0; x<contours.size(); x++){
            cv::approxPolyDP(cv::Mat(contours[x]), approx, cv::arcLength(cv::Mat(contours[x]), true)*0.02, true);
            if (std::fabs(cv::contourArea(contours[x])) < 100 || !cv::isContourConvex(approx)){
                continue;
            }else{
                    if(_markers[j]->pat.vertexnum != approx.size()){
                       continue;
                    }
                    else{

                         shape foundshape;
                         foundshape.vertexnum = approx.size();
                         foundshape.contours = contours[x];
                         for (int y = 2; y < foundshape.vertexnum+1; y++)
                            foundshape.cosines.push_back(getCosine(approx[y%foundshape.vertexnum], approx[y-2], approx[y-1]));

                         std::sort(foundshape.cosines.begin(), foundshape.cosines.end());
                         foundshape.mincos = foundshape.cosines.front();
                         foundshape.maxcos = foundshape.cosines.back();
                         foundshapes[i].push_back(foundshape);

                    }
                }
            }
        }
    }
}


void IRMA::getMarkers(){

    for(int m=0; m<_markers.size(); m++){
        _markers[m]->infoundcams = 0;
        syspoints.foundinbasecam[m] = false;
    }

    int s;
    for(int i=0; i<_cameras.size(); i++){
        shapes[i].clear();
        for(int m=0; m<_markers.size(); m++){
            s=0;
            while(s < foundshapes[i].size() ){
                if(sysmarkerparameters.mandatoryangles==true){
                    if(foundshapes[i][s].vertexnum == _markers[m]->pat.vertexnum  && foundshapes[i][s].mincos >= _markers[m]->pat.mincos  && foundshapes[i][s].maxcos <= _markers[m]->pat.maxcos){
                        cv::Rect boundingrect = cv::boundingRect(foundshapes[i][s].contours);
                        cv::Point centroid(boundingrect.x + (boundingrect.width * 0.5) , boundingrect.y + (boundingrect.height *  0.5));
                        double angle = acos(getCosine(foundshapes[i][s].contours[1],foundshapes[i][s].contours[0],centroid)) * 180.0 / PI;

                        syspoints._markers_pos_pt[i][m] = centroid;
                        syspoints._markers_rot_pt[i][m] = angle;

                            if(i == sysbase.basecam )
                                syspoints.foundinbasecam[m] = true;


                        _markers[m]->infoundcams++;

                        shapes[i].push_back(foundshapes[i][s]);

                        std::cout << "marker " <<m << " found in camera["<<i<<"] in "<<centroid<< " with angle="<<angle << std::endl;
                        m++;
                        s = 999999;

                    }else{
                        if(sysmarkerparameters.restablishpositions==true){
                            syspoints._markers_pos_pt[i][m] = {OUTPOINT,OUTPOINT};
                            syspoints._markers_rot_pt[i][m] = 0;
                        }

                    }
                }else{
                    if(foundshapes[i][s].vertexnum == _markers[m]->pat.vertexnum  ){

                        cv::Rect boundingrect = cv::boundingRect(foundshapes[i][s].contours);
                        cv::Point centroid(boundingrect.x + (boundingrect.width * 0.5) , boundingrect.y + (boundingrect.height *  0.5));
                        double angle = acos(getCosine(foundshapes[i][s].contours[1],foundshapes[i][s].contours[0],centroid)) * 180.0 / PI;

                        syspoints._markers_pos_pt[i][m] = centroid;
                        syspoints._markers_rot_pt[i][m] = angle;
                            if(i == sysbase.basecam )
                                syspoints.foundinbasecam[m] = true;


                        _markers[m]->infoundcams++;

                        foundshapes[i][s].type = m;
                        shapes[i].push_back(foundshapes[i][s]);

                        //std::cout << "marker " <<m << " found in camera["<<i<<"] in "<<centroid<< " with angle="<<angle << std::endl;
                        m++;
                        s = 999999;

                    }else{
                        if(sysmarkerparameters.restablishpositions==true){
                            syspoints._markers_pos_pt[i][m] = {OUTPOINT,OUTPOINT};
                            syspoints._markers_rot_pt[i][m] = 0;

                        }
                    }
                }
                s++;
            }
        }
        foundshapes[i].clear();
    }

}
void IRMA::getOrigin(){
    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Point> approx;

    for(int i=0; i<_cameras.size(); i++){
        cv::findContours(sysmomentum._frames_binary[i], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        for(int x=0; x<contours.size(); x++){
            cv::approxPolyDP(cv::Mat(contours[x]), approx, cv::arcLength(cv::Mat(contours[x]), true)*0.02, true);
            double area = cv::contourArea(contours[x]);
            cv::Rect boundingrect = cv::boundingRect(contours[x]);
            int radius = boundingrect.width / 2;

            if (contours[x].size() > 15 && std::abs(1 - ((double)boundingrect.width / boundingrect.height)) <= 0.2 && std::abs(1 - (area / (PI * std::pow(radius, 2)))) <= 0.2){
                    cv::Point centroid(boundingrect.x + (boundingrect.width * 0.5) , boundingrect.y + (boundingrect.height *  0.5));
                    syspoints._origin_pos_pt[i] =centroid;

                     if(i == sysbase.basecam )
                        syspoints.originfoundinbasecam = true;


                        _origin->infoundcams++;

            }
        }
    }
}


void IRMA::detectMarkers3D(){//calcula systempoints._eucl3d_ps[i] a partir de la funcio de TriangulateMarkerPoints

    for(int i=0; i<_markers.size(); i++){
        if(_markers[i]->infoundcams >= 2){ //found in at least 2 cameras
            TriangulateMarkerPoints(sysbase.basecam,i);

            //std::cout << "Triangulated (homogeneous) marker " <<i << " to "<< _markers[i]->_homo3d_ps<< std::endl;

            if(_markers[i]->_homo3d_ps.at<int>(0,0) != OUTPOINT)
                setMarkersHomoPointToEuclideanPoint(i);

           // std::cout << "Converted (euclidean) marker " <<i << " to "<< _markers[i]->_eucl3d_ps<< std::endl;

            if(_origin->infoundcams == -1)
                set3DWorld(i);

            //std::cout << "Origin= " << _origin->_eucl3d_ps<< std::endl;
            //std::cout << "Converted (world) marker " <<i << " to "<< _markers[i]->_eucl3d_ps<< std::endl;

            _markers[i]->restablishcount = 10;
        }else{
            //conservar el anterior
            _markers[i]->restablishcount--;
            if(_markers[i]->restablishcount <=0){
                _markers[i]->_eucl3d_ps.at<double>(0,0) = OUTPOINT;
                 _markers[i]->_eucl3d_ps.at<double>(0,1) = OUTPOINT;
                  _markers[i]->_eucl3d_ps.at<double>(0,2) = OUTPOINT;
                _markers[i]->_homo3d_ps.at<double>(0,0) = OUTPOINT;
                 _markers[i]->_homo3d_ps.at<double>(0,1) = OUTPOINT;
                  _markers[i]->_homo3d_ps.at<double>(0,2) = OUTPOINT;
                  _markers[i]->_homo3d_ps.at<double>(0,3) = OUTPOINT;
            }
        }
    }

}


