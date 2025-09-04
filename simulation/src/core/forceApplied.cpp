#include <array>

#include "../../include/core/vectorMath.h"

namespace SimCore{
//origin of lever vector is the center of gravity.
//***NOT the absolute cordinates.***
std::array<float , 3> forceToMoment(std::array<float,3> forceVector , std::array<float,3> leverVector , float appliedForceDistanceToCg){
    
    std::array<float,3> normalMomentArm = normalizeVector(leverVector);
    std::array<float,3> moments;
    std::array<float,3> momentArm;

    for(int i = 0 ; i<3 ; i++){
        momentArm[i] = normalMomentArm[i] * appliedForceDistanceToCg;
    }

    vectorCrossProduct(momentArm,forceVector,moments);
   
    return moments;

}

std::array<float , 3> forceToMoment(std::array<float,3> forceVector , std::array<float,3> leverVector ){
    
    std::array<float,3> moments;

    vectorCrossProduct(leverVector,forceVector,moments);
   
    return moments;

}



}





