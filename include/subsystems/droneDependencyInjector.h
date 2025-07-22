
#ifndef DRONEDEPENDENCYINJECTGOR_H
#define DRONEDEPENDENCYINJECTGOR_H

#include "motor.h"
#include "propeller.h"

namespace SimCore{
using propMotorPair  = std::pair<std::vector<std::unique_ptr<motor>>,std::vector<std::unique_ptr<propeller>>>;

//Set Square allows the creation of a rectagular prop profile.
//positive x = front , positive y = right, positive Z = top
// prop is a basis object location will be overwritten in setSquare
propMotorPair setSquare(float x, float y, propeller& prop, motor& mot);


}

#endif