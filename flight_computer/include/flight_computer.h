#include "../telemetry/include/communication_base.h"
#include "../telemetry/include/telemetry_packet.h"
#include "../state_estimation/include/state_estimation_base.h"
#include <base/base.h>
#include <util/util.h>
#include <buffer_types.h>
#include "flight_controller_base.h"
#include "scheduler_tasks.h"
#include "../hardware/include/hardware_base.h"
#include <string>
#include <iostream>
#include <cstring>
#include <spdlog/sinks/stdout_color_sinks.h>


#pragma once
namespace avionics{

class flight_computer : public thread_manager{
private:
    std::shared_ptr<controller_base> controller;

    std::shared_ptr<telemetry_base> communication;

    std::shared_ptr<estimation::estimation_base> estimator;

    std::shared_ptr<hardware_base> hardware;
    
    // ------------- buffers --------------
    
    avionics::telem_ring controller_to_communication;
        
    avionics::state_dbuf estimator_to_controller;

    avionics::sensor_dbuf hardware_to_estimator;


    void thread_process() override;

    void thread_startup_process() override;
    
public:
    

    template<typename T, typename... Args>
    void set_controller(Args&&... args){

        static_assert(std::is_base_of_v<controller_base, T>);

        controller = std::make_shared<T>(   
            estimator_to_controller, 
            controller_to_communication, 
            std::forward<Args>(args)...
        );
        controller->set_logger(logger);

    }

    template<typename T , typename... Args>
    void set_telemetry(Args&&... args){

        static_assert(std::is_base_of_v<telemetry_base,T>);

        communication = std::make_shared<T>(
            controller_to_communication,
            std::forward<Args>(args)...
        );
        communication->set_logger(logger);

    }

    template<typename T , typename... Args>
    void set_estimator(Args&&... args){

        static_assert(std::is_base_of_v<estimation::estimation_base,T>);

        estimator = std::make_shared<T>(
            hardware_to_estimator,
            estimator_to_controller,
            std::forward<Args>(args)...
        );
        estimator->set_logger(logger);

    }

    template<typename T, typename... Args>
    void set_hardware(Args&&... args){
        static_assert(std::is_base_of_v<hardware_base, T>);
        hardware = std::make_shared<T>(
            hardware_to_estimator,
            std::forward<Args>(args)...
        );
        hardware->set_logger(logger);
    }

    flight_computer(float interval_ms = 10);   

    void set_logger(std::shared_ptr<spdlog::logger> shared_logger) override;
    

};

}
