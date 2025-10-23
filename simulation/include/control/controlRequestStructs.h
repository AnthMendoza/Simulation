#pragma once
#include <vector>
#include <array>
#include "../core/quaternion.h"
#include "../core/coordinateSystem.h"

//different packets can be used for motor + control surfaces
namespace controlPacks{

struct motorOnlyPacket{
    std::vector<float> thrust;
};


using variantPackets = std::variant<motorOnlyPacket>;



struct forceMoments{
    SimCore::threeDState force;
    SimCore::threeDState moments;   
    SimCore::poseState referencePose;

    forceMoments(): force({0,0,0}),
                    moments({0,0,0}),
                    referencePose(CoordinateSystem::WORLD_BASIS){
    }
};

}