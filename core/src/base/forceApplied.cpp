#include <array>

#include "../include/base/vectorMath.h"
#include <base/units.h>

namespace SimCore{
//origin of lever vector is the center of gravity.
//***NOT the absolute cordinates.***
units::vec3 forceToMoment(units::vec3 forceVector , units::vec3 leverVector , float appliedForceDistanceToCg){
    
    units::vec3 normalMomentArm = normalizeVector(leverVector);
    units::vec3 moments;
    units::vec3 momentArm;

    for(int i = 0 ; i<3 ; i++){
        momentArm[i] = normalMomentArm[i] * appliedForceDistanceToCg;
    }

    vectorCrossProduct(momentArm,forceVector,moments);
   
    return moments;

}

units::vec3 forceToMoment(units::vec3 forceVector , units::vec3 leverVector ){
    
    units::vec3 moments;

    vectorCrossProduct(leverVector,forceVector,moments);
   
    return moments;

}



}





