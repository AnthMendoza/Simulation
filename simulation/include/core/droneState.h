#pragma once 

#include <vector>
#include <array>
#include "quaternion.h"

namespace SimCore{
namespace InteropLayer{
struct droneConfiguration{
    std::vector<threeDState> nominalLocation;
    std::vector<threeDState> nominalPropDirection;
    float mass = 0.0f;
};

struct statePacket{
    threeDState position;
    threeDState velocity;
    float timeStamp;
    poseState pose;
    std::vector<float> motorAngularVelocity;
    droneConfiguration config;
};

}
}