#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include "inc_Drivers.h"

#include <sstream>
#include <exception>
#include <iostream>


class UeyeException : public std::exception {
private:
        HIDS cam;
        int exceptionId;
public:

        UeyeException(HIDS cam, int err) {
                exceptionId = err;
        }

        const char * what() const throw () {
                std::stringstream ss;
                ss << "UeyeException on camera " << cam <<", with exit code:\t" << exceptionId;
                return ss.str().c_str();
        }

        HIDS getCam()
        {
                return cam;
        }

        int getExceptionId()
        {
                return exceptionId;
        }
};

#endif // EXCEPTIONS_H
