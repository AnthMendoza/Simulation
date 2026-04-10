#include "../include/flight_computer.h"

avionics::flight_computer::flight_computer(float interval_ms): thread_manager(interval_ms){
    set_logger(spdlog::stdout_color_mt("flight_controller"));
}

void avionics::flight_computer::set_logger(std::shared_ptr<spdlog::logger> shared_logger){
    thread_manager::set_logger(std::move(shared_logger));

    spdlog::set_pattern("[%H:%M:%S.%e] [%t] [%n] [%^%l%$] %v");

    if (hardware) {
        hardware->set_logger(logger);
    }

    if (estimator) {
        estimator->set_logger(logger);
    }

    if (controller) {
        controller->set_logger(logger);
    }

    if (communication) {
        communication->set_logger(logger);
    }
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
