#ifndef AVIONICS_STATES_H
#define AVIONICS_STATES_H
#include <array>

namespace avionics{

    struct estimated_state{

        std::array<float,3> position;
        std::array<float,3> velocity;
        std::array<float,3> acceleration;
        std::array<float,3> direction_vector;

    };

    struct desired_state{

        std::array<float,3> position;
        std::array<float,3> velocity;

        desired_state(): position({0,0,0}),velocity({0,0,0}){
        }
    };

    struct controller_feedback{



    };

}

#endif