#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "inc_CV.h"

#define PI 3.1416
#define OUTPOINT -999




struct board{ //our callibration board
    int width;
    int height;
    float entitysize;

    int ncaptures;

};

struct blobdetectorparameters : public cv::SimpleBlobDetector::Params{

};

struct IRMAmarkerparameters{

    //canny & blur
    int minThreshold;
    int maxThreshold;
    int aperturesize;
    int kerneldims;

    //marker stuff
    bool mandatoryangles;
    bool restablishpositions;
};
/////////////////////////////////////////



/////////////////////////////////////////
struct calibrateparameters{ //obtained from every good board frame captures

    std::vector< std::vector<cv::Point3f> >  objectPoints;
    std::vector< std::vector<cv::Point2f> > imagePoints;

};

struct intrinsicparameters{ //obtained from callibration or stereo callibration
    cv::Mat distCoeffs;
    cv::Mat cameramatrix;
    cv::Mat distCoeffsOther;
    cv::Mat cameramatrixOther;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
};

struct extrinsicparameters{ //obtained from stereo callibration
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;
};

struct relativeparameters{ //obtained from stereo rectification
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;
};
//////////////////////////////////////////

struct IRMAbase{

    int camsnum;
    int patnum;
    std::string cameratype;
    int basecam;

    int width;
    int height;



};

struct IRMAmomentum{

    std::vector<cv::Mat> _frames;
    std::vector<std::vector<cv::Mat>> _frames_board_captured;
    std::vector<cv::Mat> _frames_undistorted;
    std::vector<cv::Mat> _frames_binary;

    std::vector<cv::Mat> _frames_blobbed;
    std::vector<cv::Mat> _frames_marked;

    std::vector<cv::Mat> _frames_show;

    bool drawcanny;
    bool drawblobs;
    bool drawmarkers;
    bool drawmarkers3D;
};

struct IRMAfileconfig{

    char* directory;
    char* configpath;
    char* configcampath;
    char* configmarkerpath;

};


struct IRMAstatus{

    int status;
    //-1 no camera connected
    //0 all cameras connected
    //1 all cameras calibrated
    //2 all cameras stereocalibrated
    //3 all cameras stereorectified
    std::vector<int> camerastatus;
    //-1 disconnected
    //0 connected
    //1 calibrated (has intrinsic parameters), able to undistort
    //2 stereocalibrated (has intrinsic and extrinsic parameters), able to undistort & rectify
    //3 stereorectified (has intrinsic, extrinsic and relative parameters), able to triangulate
};


struct IRMAparameters{ //for each cam

    std::vector<calibrateparameters> camcalibrateparameters;
    std::vector<intrinsicparameters> camintrinsicparameters;
    std::vector<extrinsicparameters> camextrinsicparameters;
    std::vector<relativeparameters> camrelativeparameters;
};

struct IRMApatterns{
    std::vector<pattern> patterns;
};

struct IRMApoints{

    std::vector<std::vector<cv::KeyPoint> > _blobs;

    std::vector<std::vector<cv::Point> > _markers_pos_pt; //by default there are N (number of patterns), set on position 0,0
    std::vector<std::vector<double > > _markers_rot_pt; //by default there are N (number of patterns), set on rotation 0,0
    std::vector<cv::Point> _origin_pos_pt;

    std::vector<bool> foundinbasecam;
    bool originfoundinbasecam;
};

struct IRMAworld{

    cv::Mat _3D_WORLD;
};


//////////////////////////////////////////


struct shape{
    int type;
    int vertexnum;
    std::vector<double> cosines;
    std::vector<cv::Point> contours;

    double mincos;
    double maxcos;
};


#endif // PARAMETERS_H
