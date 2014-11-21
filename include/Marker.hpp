#ifndef MARKER_H
#define MARKER_H

#include "inc_CV.h"
#include "inc_GL.h"


struct pattern{
    int id;
    int vertexnum;
    double mincos;
    double maxcos;

};



#include <parameters.h>


class Marker
{
    public:
        Marker();

        virtual ~Marker();

        pattern pat;

        cv::Mat _homo3d_ps;
        cv::Mat _homo3d_rot;

        cv::Mat _eucl3d_ps;
        cv::Mat _eucl3d_rot;

        int infoundcams;
        int restablishcount;

        GL::Object* polygon;

    protected:
    private:


};

#endif // MARKER_H
