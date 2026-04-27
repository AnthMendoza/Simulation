#pragma once

#include <buffer/ring_buffer.h>
#include <buffer/double_buffer.h>
#include <actual_state.h>
#include <sensor_field.h>
#include <telemetry_packet.h>
#include <avionics_states.h>
#include <sensor_field.h>
#include <computer_state.h>

namespace avionics{

// ------------- controller --> telemetry --------------

using flight_telemetry = avionics::flight_controller_telemetry_packet;

using telem_ring = utility::buffer::ring_buffer<flight_telemetry,20>;

// ------------- estimation --> controller --------------

using estimation_state_packet = avionics::estimated_state;
        
using state_dbuf = utility::buffer::double_buffer<estimation_state_packet>;

// ------------- hardware --> estimation --------------

using sensor_field_packet = avionics::sensor::SensorField;

using sensor_dbuf = utility::buffer::double_buffer<sensor_field_packet>;

// ------------- navigation --> controller --------------

using desired = desired_state;

using nav_ring = utility::buffer::ring_buffer<desired,20>;

// ------------- flight_computer_global_flags --------------

using flight_health = avionics::flight_computer_health;

}