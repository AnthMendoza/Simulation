#include "../include/flight_computer.h"

avionics::flight_computer::flight_computer(float interval_ms): thread_manager(interval_ms){

}


void avionics::flight_computer::thread_proccess(){

}

void avionics::flight_computer::thread_startup_proccess(){

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

    if(estimator){
        estimator->start();
    }else{
        std::cerr<< "\nFlight Computer contains no estimator\n";
    }
}
