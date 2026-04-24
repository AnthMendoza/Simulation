#include "../include/flight_computer.h"



avionics::airframe_config make_flight_controller_config(const std::string& airframeConfigPath) {
    if (airframeConfigPath.empty()) {
        return avionics::airframe_config();
    }

    auto parsed = avionics::airframe_config_parser::fromFile(airframeConfigPath);

    return parsed;
}


avionics::flight_computer::flight_computer(float interval_ms, std::string airframeConfigPath)
    : thread_manager(interval_ms) {
    set_logger(spdlog::stdout_color_mt("flight_controller"));
    configuration = config_startup(airframeConfigPath);
    logger->set_level(spdlog::level::debug);
    configuration.print_debug(logger);

}


avionics::airframe_config avionics::flight_computer::config_startup(const std::string& airframeConfigPath) {
    
    return make_flight_controller_config(airframeConfigPath);
}

void avionics::flight_computer::set_logger(std::shared_ptr<spdlog::logger> shared_logger){
    thread_manager::set_logger(std::move(shared_logger));
    //adds thread PID to comment
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
    auto& nav_buffer = controller->get_nav_buffer();
    desired test_state;
    test_state.position = {1,0,1};
    nav_buffer.push(test_state);
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