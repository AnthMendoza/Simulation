#ifndef FORCEAPPLIED_H
#define FORCEAPPLIED_H

#include <array>

namespace SimCore{

units::vec3 forceToMoment(units::vec3 forceVector , units::vec3 vehicleState , float appliedForceDistanceToCg);

units::vec3 forceToMoment(units::vec3 forceVector , units::vec3 leverVector );


void finForce();


}



#endif