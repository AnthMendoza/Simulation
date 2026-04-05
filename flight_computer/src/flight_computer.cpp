#include "../include/flight_computer.h"

avionics::flight_computer::flight_computer(float interval_ms): thread_manager(interval_ms){

}


void avionics::flight_computer::thread_process(){

}

void avionics::flight_computer::thread_startup_process(){

    if(hardware){
        hardware->start();
    }else{
        std::cerr<< "\nFlight Computer contains no hardware\n";
    }

    if(estimator){
        estimator->start();
    }else{
        std::cerr<< "\nFlight Computer contains no estimator\n";
    }

    if(controller){
        controller->start();
    }else{
        std::cerr<< "\nFlight Computer contains no controller\n";
    }

    if(communication){
        communication->start();
    }else{
        std::cerr<< "\nFlight Computer contains no communication\n";
    }

    
}
