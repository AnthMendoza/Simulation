#ifndef PIDFUNCTIONGROUP_H
#define PIDFUNCTIONGROUP_H

#include <tuple>

namespace SimCore{

//Kp,Ki,Kd respectively
using dataPID = std::tuple<float,float,float>;
using PIDPair = std::pair<dataPID,dataPID>;

class droneBody;
class droneControl;
class calibratePID;

template<typename PIDType>
struct PIDFunctionGroup {
    bool (*iteration)(droneBody*, calibratePID*);
    void (*setUp)(droneBody*, PIDDroneController*, PIDType);
    void (*setGains)(PIDDroneController*, PIDType);
    float stdThreshold;
    float errorThreshold;
    float maxDuration;
};

}

#endif