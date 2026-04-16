#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include "communication_base.h"
#include <iostream>


namespace avionics{

class telemetry: public telemetry_base{

    private:

    protected:

    std::vector<avionics::scheduler_tasks> get_routine() override;

    #define DEFINE_SENDS(TYPE,FIELD)\
    void send_##TYPE##_packet(){\
        send(packet_manager.get_##TYPE##_packet());\
    }


    DEFINE_SENDS(ATTITUDE, attitude)
    DEFINE_SENDS(POSITION, position)
    DEFINE_SENDS(VELOCITY, velocity)
    DEFINE_SENDS(STATUS, status)
    DEFINE_SENDS(ERROR, error)

    #undef DEFINE_SENDS

    public:

    telemetry(telem_ring& tel_buff , std::string config_path) : telemetry_base(tel_buff,config_path){

    }



};


}

#endif 