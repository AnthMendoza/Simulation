#include "../../include/sim/hardware_sim.h"

using namespace avionics::sim;

void hardware_sim::set_field_sim(avionics::sensor::SensorField sim_field){

}

void hardware_sim::set_field(){
    auto write = sensor_buffer.writeBuffer();
    write = bus.read_sensor();
    sensor_buffer.publish();
}


void hardware_sim::thread_process(){
    set_field_sim(bus.read_sensor());
}

void hardware_sim::thread_startup_process(){

}
