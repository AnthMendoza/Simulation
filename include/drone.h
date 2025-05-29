#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"
#include "motor.h"
#include "battery.h"
#include <utility>
#include <memory>
#include <string>
using namespace std;
namespace SimCore{
class droneBody :  public Vehicle{
    private:
    void motorThrust(float motorRPM);
    //prop locations in relation to the center of gravity
    vector<array<float,3>> propLocations;
    //prop force vector allows for unconventional motor mounting
    vector<array<float,3>> propForceVector;
    // .first is current and .second is previous rpm
    vector<pair<float,float>> propRPM;
    vector<float> propMOI;
    //true if props are already formed
    bool propLocationsSet;

    array<float,3> cogLocation;
    //location of motor is logged via its index in vector and the propLocatiion vector
    vector<std::unique_ptr<motor>> motors;
    //droneBattery
    std::unique_ptr<battery> droneBattery;
    protected:

    public:
    droneBody();
    ~droneBody();
    droneBody(const droneBody& drone) = delete;
    void updateState() override; 
    void init(string& motorConfig, string& batteryConfig);
    //Set Square allows the creation of a rectagular prop profile.
    //positive x = front , positive y = right, positive Z = top
    void setSquare(float x , float y , float propellerMOI);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(array<float,3> offset);

    void motorMoment();

    void thrustRequest(vector<float>& thrust);

    std::pair<vector<array<float,3>>, vector<array<float,3>>>& transposedProps();

    inline battery* getBattery(){
        battery* bat = dynamic_cast<battery*> (droneBattery.get()); 
        return bat;
    }

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
} //SimCore


#endif