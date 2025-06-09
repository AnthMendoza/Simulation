#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"
#include "motor.h"
#include "battery.h"
#include "quaternion.h"
#include "indexVectors.h"
#include "PIDController.h"
#include "propeller.h"
#include <utility>
#include <memory>
#include <string>
using namespace std;
namespace SimCore{

class droneBody :  public Vehicle{
    private:
    void motorThrust(float motorRPM);

    vector<float> thrustRequestVect;
    //true if props are already formed
    bool propLocationsSet;
    int transposeCalls;
    //index vectorsßßß

    array<float,3> cogLocation;
    array<float,3> cogLocationTranspose;
    //location of motor is logged via its index in vector and the propLocatiion vector
    vector<unique_ptr<motor>> motors;
    vector<unique_ptr<propeller>> propellers;
    //droneBattery
    unique_ptr<battery> droneBattery;

    unique_ptr<quaternionVehicle> pose;
    indexCoordinates index;
    //helper Functions
    void rotationHelper(Quaternion& q);

    void resetHelper();

    protected:
    array<float,3>  thrustVector();
    vector<float> thrust();
    public:
    droneBody();
    ~droneBody();
    string droneConfig;
    //droneBody(const droneBody& drone) = delete;
    void updateState() override; 
    void init(string& motorConfig ,string& batteryConfig , string& droneBody);
    void setSquare(float x , float y , propeller prop);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(array<float,3> offset);

    void motorMoment();

    inline void thrustRequest(vector<float>& thrust){
        thrustRequestVect = thrust;
    }

    void transposedProps();

    inline battery* getBattery(){
        battery* bat = dynamic_cast<battery*> (droneBattery.get()); 
        return bat;
    }

};

//Gives a high level state request that is handled down stream
//Container for Drone body
class droneControl{
    private:
    unique_ptr<PIDController> PIDX;
    unique_ptr<PIDController> PIDY;
    unique_ptr<PIDController> PIDZ;
    protected:
    public:
    unique_ptr<droneBody> body;
    droneControl();
    //initialization out of constructor due to out of order calls in unreal engine.
    
    void init(string& motorConfig, string& batteryConfig , string& droneConfig); 
    void initpidControl();
    array<float , 3> pidControl(float x , float y, float z);
    void setpidControl(float xTarget , float yTarget , float zTarget);
};
} //SimCore


#endif  