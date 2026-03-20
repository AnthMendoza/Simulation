#include "../../simulation/include/sim/toml.h"
#include "../../simulation/include/utility/utility.h"
#include "../telemetry/include/communication_base.h"
#include "../telemetry/include/telemetry_packet.h"
#include "../state_estimation/include/state_estimation_base.h"
#include "../include/thread_manager.h"
#include "flight_controller_base.h"
#include "../utility/include/ring_buffer.h"
#include "../utility/include/double_buffer.h"
#include "scheduler.h"
#include "scheduler_tasks.h"
#include <string>
#include <iostream>
#include <cstring>


#pragma once
namespace avionics{

class flight_computer : public thread_manager{
private:
    std::shared_ptr<controller_base> controller;

    std::shared_ptr<telemetry_base> communication;

    std::shared_ptr<estimation::estimation_base> estimator;


    utility::buffer::ring_buffer<avionics::flight_controller_telemetry_packet,20> controller_to_communication;
        
    utility::buffer::double_buffer<avionics::estimated_state> estimator_to_controller;

    


    void thread_proccess() override;

    void thread_startup_proccess() override;
    
public:

    template<typename T, typename... Args>
    void set_controller(Args&&... args){

        static_assert(std::is_base_of_v<controller_base, T>);

        controller = std::make_shared<T>(std::forward<Args>(args)...);

    }

    template<typename T , typename... Args>
    void set_telemetry(Args&&... args){

        static_assert(std::is_base_of_v<telemetry_base,T>);

        communication = std::make_shared<T>(std::forward<Args>(args)...);

    }

    template<typename T , typename... Args>
    void set_estimator(Args&&... args){

        static_assert(std::is_base_of_v<estimation::estimation_base,T>);

        estimator = std::make_shared<T>(std::forward<Args>(args)...);

    }

    flight_computer(float interval_ms = 1);    

};

}