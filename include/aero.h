#ifndef AERO_H
#define AERO_H
namespace SimCore{
    float airDensity(float Zposition);

    float aeroAreaRocket(float angle);

    float coefOfDragRocket(float angle);
    
    float coefOfLiftRocket(float angle);

    float aeroAreaDrone(float angle);

    float coefOfDragDrone(float angle);
    
    float coefOfLiftDrone(float angle);

}
    
#endif