IRMA
====
* Setup instructions [here](https://github.com/dberga/IRMA/blob/master/INSTALL.txt)
* Credits [here](https://github.com/dberga/IRMA/blob/master/CREDITS.txt)
* License [here](https://github.com/dberga/IRMA/blob/master/LICENSE)
* Demo video [here](https://www.youtube.com/watch?v=Y77tmCQwrN4)

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Y77tmCQwrN4/0.jpg)](https://www.youtube.com/watch?v=Y77tmCQwrN4)

___

InfraRed MArkers Library (IRMA)

Version 1.1
___
InfraRed MArkers Library is a computer vision development library designed
to provide the detection of 3D coordinates from geometric shape markers using
infrared cameras as uEye.

IRMA supports uEye infrared cameras but also any kind of camera 
using the "VideoCaptureCamera" from openCV VideoCapture class. 

IRMA 1.1 has been tested in Windows x64 builded with mingw. IRMA is written in C++ 
and uses [pugi::xml](http://pugixml.org/), [opencv libraries](http://opencv.org/), [opengl](https://www.opengl.org/) GUI and the [uEye API](http://en.ids-imaging.com/download-ueye.html) for specific uEye cameras

This library is distributed under the New BSD license, which can be found
in the file "LICENSE.txt".

This library also includes its demo executable which uses IRMA base functions,
able to draw an identificator of every marker, each marker contour, 3D coordinates, origin and infrared blobs.
