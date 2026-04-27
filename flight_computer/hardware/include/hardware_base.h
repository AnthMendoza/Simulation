#ifndef HARDWARE_BASE_H
#define HARDWARE_BASE_H

#include <sensor_field.h>
#include <util/thread_manager.h>
#include <mutex>
#include <buffer_types.h>
#include <iostream>
#include <subsystem_monitor.h>

namespace avionics{


class hardware_base : public thread_manager , subsystem<&flight_computer_health::hardware>{
    protected:
    sensor::SensorField field;
    sensor_dbuf& sensor_buffer;
    flight_health& health_buffer;

    public:
    
    //hard code polling change to schedule driven dependent on sensor rates
    
    hardware_base(flight_health& flight_health,sensor_dbuf& sensor_buff): health_buffer(flight_health) , sensor_buffer(sensor_buff) , thread_manager(10) , subsystem<&flight_computer_health::hardware>(flight_health){

    }

    virtual void set_field() = 0;

    virtual sensor::SensorField get_field();

    void thread_startup_process() override{
        SPDLOG_LOGGER_INFO(logger, "Hardware started");
    }
    
};

}

#endif