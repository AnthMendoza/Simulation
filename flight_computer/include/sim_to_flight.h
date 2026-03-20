#pragma once
#include"../utility/include/double_buffer.h"

namespace avionics{

template<typename SensorField, typename ActualState>

class sim_to_flight {
private:
    utility::buffer::double_buffer<SensorField>  sensor_buffer;
    utility::buffer::double_buffer<ActualState>  actual_state_buffer;
public:
    void sim_publish(const ActualState& state, const SensorField& field){
        actual_state_buffer.writeBuffer() = state;
        actual_state_buffer.publish();
        sensor_buffer.writeBuffer() = field;
        sensor_buffer.publish();
    }

    
    void hardware_publish(const SensorField& field){
        sensor_buffer.writeBuffer() = field;
        sensor_buffer.publish();
    }

    SensorField read_sensor() const{
        return sensor_buffer.read();
    }

   
    ActualState read_actual() const{
         return actual_state_buffer.read();
        }

};

}