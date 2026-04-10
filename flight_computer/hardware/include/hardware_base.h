#ifndef HARDWARE_BASE_H
#define HARDWARE_BASE_H

#include <sensor_field.h>
#include <util/thread_manager.h>
#include <mutex>
#include <buffer_types.h>
#include <iostream>

namespace avionics{


class hardware_base : public thread_manager{
    protected:
    sensor::SensorField field;
    sensor_dbuf& sensor_buffer;


    public:
    
    //hard code polling change to schedule driven dependent on sensor rates
    
    hardware_base(sensor_dbuf& sensor_buff): sensor_buffer(sensor_buff) , thread_manager(10){

    }

    virtual void set_field() = 0;

    virtual sensor::SensorField get_field();

    void thread_startup_process() override{
        SPDLOG_LOGGER_INFO(logger, "Hardware started");
    }
    
};

}

#endif