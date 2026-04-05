#pragma once

#include <hardware_base.h>
#include "../sim/sim_to_flight.h"
#include <sensor_field.h>
#include <actual_state.h>

namespace avionics::sim{

class hardware_sim:public avionics::hardware_base{
    private:

    using bus_type = avionics::sim_to_flight<avionics::sensor::SensorField,avionics::sensor::actual_state>;
    bus_type& bus;

    public:

    hardware_sim(sensor_dbuf& sensor_buff , bus_type& bus_): hardware_base(sensor_buff) , bus(bus_){
    }
    
    void set_field() override;

    void set_field_sim(avionics::sensor::SensorField sim_field);

    void thread_process() override;

    void thread_startup_process() override;

};

}