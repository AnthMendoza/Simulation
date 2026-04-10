#pragma once 

#include <vector>
#include <array>
#include "quaternion.h"
#include <base/units.h>

namespace SimCore{
namespace InteropLayer{
struct droneConfiguration{
    std::vector<units::vec3> nominalLocation;
    std::vector<units::vec3> nominalPropDirection;
    float mass = 0.0f;
};

struct statePacket{
    units::vec3 position;
    units::vec3 velocity;
    float timeStamp;
    poseState pose;
    std::vector<float> motorAngularVelocity;
    droneConfiguration config;
};

}
}