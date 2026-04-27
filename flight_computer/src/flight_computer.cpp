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
    manager_health();

    auto& nav_buffer = controller->get_nav_buffer();
    desired test_state;
    test_state.position = {1,0,1};
    nav_buffer.push(test_state);
}

void avionics::flight_computer::thread_startup_process(){

    auto safeStart = [](auto& component, const char* name){
        if (!component) {
            std::cerr << "\nFlight Computer contains no " << name << "\n";
            return false;
        }

        try{
            component->start();
            return true;
        } catch (const std::exception& error){
            std::cerr << "\nFailed to start " << name << ": " << error.what() << "\n";
            return false;
        }
    };

    bool safe =
        safeStart(hardware, "hardware") &&
        safeStart(estimator, "estimator") &&
        safeStart(controller, "controller") &&
        safeStart(communication, "communication");

    if (!safe){
        //set controller flag to prevent motor torque commands

        throw std::runtime_error("Abort");
    }
    
}



void avionics::flight_computer::manager_health(){

}