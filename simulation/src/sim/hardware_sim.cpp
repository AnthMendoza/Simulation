#include "../../include/sim/hardware_sim.h"

using namespace avionics::sim;



void hardware_sim::set_field(){
    field = bus.read_sensor();
    sensor_buffer.writeBuffer() = field;
    sensor_buffer.publish();
}

void hardware_sim::thread_process(){
    set_field();

    const auto published_field = sensor_buffer.read();
    if (published_field.accel[0].has_value()) {
        const auto& accel = published_field.accel[0].value().accel_mss;
    }
}

void hardware_sim::thread_startup_process(){
    hardware_base::thread_startup_process();
}
