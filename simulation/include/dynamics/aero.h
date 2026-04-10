#ifndef AERO_H
#define AERO_H

#include <base/units.h>

namespace SimCore{
    units::scalar airDensity(units::scalar Zposition);

    units::scalar aeroAreaRocket(units::scalar angle);

    units::scalar coefOfDragRocket(units::scalar angle);
    
    units::scalar coefOfLiftRocket(units::scalar angle);

    units::scalar aeroAreaDrone(units::scalar angle);

    units::scalar coefOfDragDrone(units::scalar angle);
    
    units::scalar coefOfLiftDrone(units::scalar angle);

}
    
#endif