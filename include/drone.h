#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"
#include <utility>
using namespace std;
namespace SimCore{
class droneBody :  public Vehicle{
    private:
    void motorThrust();
    //prop locations in relation to the center of gravity
    vector<array<float,3>> propLocations;
    vector<array<float,3>> propForceVector;
    // .first is current and .second is previous rpm
    vector<pair<float,float>> propRPM;
    vector<float> propMOI;
    //true if props are already formed
    bool propLocationsSet;

    array<float,3> cogLocation;
    protected:

    public:
    droneBody();
    ~droneBody();
    droneBody(const droneBody& drone) = delete;
    void updateState() override; 
    void init() override;
    //Set Square allows the creation of a rectagular prop profile.
    //positive x = front , positive y = right, positive Z = top
    void setSquare(float x , float y , float propellerMOI);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(array<float,3> offset);

    void motorMoment();

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
    void addDrone(droneBody&& body);
};
}


#endif