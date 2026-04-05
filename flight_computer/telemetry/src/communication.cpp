#include "../include/communication.h"



std::vector<avionics::scheduler_tasks> avionics::telemetry::get_routine(){

    std::vector<avionics::scheduler_tasks> tasks{
        {"terminal_print" , 4.000f, [this](float){terminal_print();}},
        {"ATTITUDE",  0.050f, [this](float) { send_ATTITUDE_packet(); }},
        {"POSITION",  0.100f, [this](float) { send_POSITION_packet(); }},
        {"VELOCITY",  0.050f, [this](float) { send_VELOCITY_packet(); }},
        {"STATUS",    3.000f, [this](float) { send_STATUS_packet(); }}
    };

    return tasks;
}