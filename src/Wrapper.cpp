#include "Wrapper.h"


Wrapper::Wrapper(){
    running = true;
}

void Wrapper::Setup(){


    //define
    _irma = new IRMA("data/");

    //config
    _irma->Config();
    _irma->WindowsCreate();
    _irma->TrackbarsCreate();
    _irma->showDevices();
    _irma->Window3DCreate();


}

void Wrapper::run_handler_iterative(){

        while(running){


            run_logic();
            run_draw();
            run_input();

        }




}

void Wrapper::run_handler(){

    //std::thread l(&Wrapper::run_logic, this);
    //std::thread d(&Wrapper::run_draw, this);
    //std::thread i(&Wrapper::run_input, this);
}

void Wrapper::run_logic(){

            _irma->iterate();


}

void Wrapper::run_draw(){

                _irma->draw();
                _irma->WindowsUpdate();
                _irma->draw3D();
}


void Wrapper::run_input(){

        int key = cv::waitKey(10);

        if (key == 27) { //ESCAPE

            stop();
        }else{

            _irma->triggerKeys(key);

        }



}
void Wrapper::stop(){

    running = false;
    std::cout << "IRMA////RUN ENDED" << std::endl;
    delete _irma;
}

