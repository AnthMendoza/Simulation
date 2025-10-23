#pragma once

#include <array>
#include "../utility/utility.h"
#include "vectorMath.h"

namespace SimCore{
struct poseState{
    std::array<float,3> dirVector;
    std::array<float,3> fwdVector;
    std::array<float,3> rightVector;
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