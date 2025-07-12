#ifndef FORCEAPPLIED_H
#define FORCEAPPLIED_H

#include <array>

namespace SimCore{

std::array<float , 3> forceToMoment(std::array<float,3> forceVector , std::array<float,3> vehicleState , float appliedForceDistanceToCg);

std::array<float , 3> forceToMoment(std::array<float,3> forceVector , std::array<float,3> leverVector );


void finForce();


}



#endif