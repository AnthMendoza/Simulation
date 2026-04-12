#ifndef AVIONICS_STATES_H
#define AVIONICS_STATES_H
#include <array>
#include <fc_units.h>

namespace avionics{

    struct estimated_state{

        fc_units::vec3 position;
        fc_units::vec3 velocity;
        fc_units::vec3 acceleration;
        fc_units::vec3 direction_vector;

    };

    struct desired_state{

        fc_units::vec3 position;
        //fc_units::vec3 velocity;

        desired_state(): position({0,0,0}){
        }
    };

    struct controller_feedback{

        fc_units::vec3 thrust_request;


    };

}

#endif