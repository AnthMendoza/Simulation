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
    void (*iteration)(droneBody*, calibratePID*);
    void (*setUp)(droneBody*, droneControl*, PIDType);
    void (*setGains)(droneControl*, PIDType);
};

}

#endif