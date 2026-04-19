#include "../include/estimation.h"
#include "../../include/avionics_states.h"



std::vector<avionics::scheduler_tasks> avionics::estimation::estimation::get_routine(){
    std::vector<avionics::scheduler_tasks> tasks{
        {"update estimation",  0.03f, [this](float) {calculate_state();}},
        {"get sensor data",  0.03f, [this](float) { get_sensor_data();}}
    };

    return tasks;
}


void avionics::estimation::estimation::calculate_state(){
   
    
    sensor_field_packet sensor_pack = sensor_buffer.read();

    const auto& gps_data = sensor_pack.gps[0];
    SPDLOG_LOGGER_INFO(
        logger,
        "GPS LAT: {}, LONG: {}, ALT: {}",
        gps_data->latitude_deg,
        gps_data->longitude_deg,
        gps_data->altitude_msl_m
    );

    const auto& accel = sensor_pack.accel[0];
    SPDLOG_LOGGER_INFO(
        logger,
        "accel : {}, {}, {}",
        accel->accel_mss[0],
        accel->accel_mss[1],
        accel->accel_mss[2]
    );

    SPDLOG_LOGGER_INFO(
        logger,
        "Velocity : {}, {}, {}",
        gps_data->velocity_ned_ms[0],
        gps_data->velocity_ned_ms[1],
        gps_data->velocity_ned_ms[2]
    );


}


void avionics::estimation::estimation::get_sensor_data(){

    
    
}