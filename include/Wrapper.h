#ifndef WRAPPER_H
#define WRAPPER_H

#include <inc_CV.h>
#include <IRMA.hpp>

#include <thread>




class Wrapper
{
    public:
        Wrapper();
        void Setup();
        void run_handler();
        void run_handler_iterative();
        void run_logic();
        void run_draw();
         void run_input();
         void stop();
    protected:
        //std::thread logic;
        //std::thread draw;
        //std::thread input;

    private:
        IRMA *_irma;
        bool running;




};

#endif // WRAPPER_H
