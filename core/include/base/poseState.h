#pragma once

#include <array>
#include "../util/utility.h"
#include "vectorMath.h"
#include <base/units.h>

namespace SimCore{
struct poseState{
    units::vec3 dirVector;
    units::vec3 fwdVector;
    units::vec3 rightVector;
    void printPose(){
        std::cout << "poseState {\n";
        print(dirVector,"  dirVector");
        print(fwdVector , "  fwdVector");
        print(rightVector, "  rightVector");
        std::cout << "}\n";
    }
    void printPose(std::string comment){
        std::cout << "poseState " << comment << " {\n";
        print(dirVector,"  dirVector");
        print(fwdVector , "  fwdVector");
        print(rightVector, "  rightVector");
        std::cout << "}\n";
    }
    void normalize(){
        normalizeVectorInPlace(dirVector);
        normalizeVectorInPlace(fwdVector);
        normalizeVectorInPlace(rightVector);
    }
};
}