#include <array>

#include "../../include/core/vectorMath.h"

namespace SimCore{

std::array<float , 3> forceToMoment(std::array<float,3> forceVector , std::array<float,3> vehicleState , float appliedForceDistanceToCg){
    
    std::array<float,3> normalMomentArm = normalizeVector(vehicleState);
    std::array<float,3> moments;
    //here we are creating a vector that repsents the length of moment arm, first we normilize our vehicle vector then we multiply it by the scaler 
    for(int i = 0 ; i<3 ; i++){
        normalMomentArm[i] = normalMomentArm[i] * appliedForceDistanceToCg;
    }

    moments[0] = normalMomentArm[1] * forceVector[2] - normalMomentArm[2] * forceVector[1];
    moments[1] = normalMomentArm[2] * forceVector[0] - normalMomentArm[0] * forceVector[2];
    moments[2] = normalMomentArm[0] * forceVector[1] - normalMomentArm[1] * forceVector[0];


    return moments;

}


}





