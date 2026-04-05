#ifndef HARDWARE_BASE_H
#define HARDWARE_BASE_H

#include <sensor_field.h>
#include <util/thread_manager.h>
#include <mutex>
#include <buffer_types.h>

namespace avionics{

class hardware_base :public thread_manager{
    protected:
    sensor::SensorField field;
    sensor_dbuf& sensor_buffer;

    public:
    
    //hard code polling change to schedule driven dependent on sensor rates
    static constexpr int poll_rate_ms = 10;
    hardware_base(sensor_dbuf& sensor_buff): sensor_buffer(sensor_buff) , thread_manager(poll_rate_ms){

    }

    virtual void set_field() = 0;

    virtual sensor::SensorField get_field();
};

}

#endif