#pragma once

#include <array>
#include <variant>
#include "../core/quaternion.h"
namespace SimCore{
//state Estimation packet to return to controller
struct stateInfo {
    std::array<float, 3> position; //x, y, z position
    poseState pose;     
    std::array<float, 3> velocity; //vx, vy, vz velocity
    std::array<float , 3> rotationalAcceleration;
    float absVelocity;             //absolute/magnitude of velocity
    float timestamp;               //time when state was captured
    
    stateInfo() 
        : position{0.0f, 0.0f, 0.0f},
          pose{0.0f, 0.0f, 0.0f},
          velocity{0.0f, 0.0f, 0.0f},
          rotationalAcceleration{0.0f, 0.0f, 0.0f},
          absVelocity(0.0f),
          timestamp(0.0f){

    }
};

struct onlyThrustAccel{
    threeDState thrustAccelerationVector;
    
    onlyThrustAccel():
    thrustAccelerationVector{0.0f, 0.0f, 0.0f}{

    }

};

//controller packet to return to state Estimation
using controllerToEstimationVariant = std::variant<onlyThrustAccel>;


}

namespace testGlobals{
namespace actualVehicleState{
    extern SimCore::stateInfo state;
}
}