#pragma once
#include <vector>
#include <array>
#include <base/base.h>
#include <fc_units.h>

//different packets can be used for motor + control surfaces
namespace controlPacks{

struct motorOnlyPacket{
    std::vector<float> thrust;
};


using variantPackets = std::variant<motorOnlyPacket>;



struct forceMoments{
    fc_units::vec3 force;
    fc_units::vec3 moments;   
    SimCore::poseState referencePose;

    forceMoments(): force({0,0,0}),
                    moments({0,0,0}),
                    referencePose(CoordinateSystem::WORLD_BASIS){
    }
};

}