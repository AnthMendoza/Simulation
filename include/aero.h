#ifndef AERO_H
#define AERO_H
namespace SimCore{
    float airDensity(float Zposition);

    float aeroArea(float angle);

    float coefOfDrag(float angle);
    
    float coefOfLift(float angle);
}
    
#endif