#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"
using namespace std;
namespace SimCore{
class droneBody :  public Vehicle{
    private:
    void motorThrust();
    protected:

    public:
    droneBody();
    ~droneBody();
    droneBody(const droneBody& drone) = delete;
    void updateState() override; 
    void init() override;
};

//Gives a high level state request that is handled down stream
class droneControl{
    private:
    unique_ptr<droneBody> drone;

    protected:
    public:
    droneControl();
    //initialization out of constructor due to out of order calls in unreal engine.
    void init();    
};
}


#endif