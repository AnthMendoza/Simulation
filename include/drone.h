#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"
#include "motor.h"
#include "battery.h"
#include "quaternion.h"
#include "indexVectors.h"
#include "PIDController.h"
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
    vector<array<float,3>> propLocationsTranspose;
    //prop force vector allows for unventional motor mounting
    vector<array<float,3>> propForceVector;
    vector<array<float,3>> propForceVectorTranspose;
    // .first is current and .second is previous rpm
    vector<float> propRPM;
    vector<float> propMOI;
    vector<float> thrustRequest;
    //true if props are already formed
    bool propLocationsSet;
    int transposeCalls;
    //index vectors

    array<float,3> cogLocation;
    array<float,3> cogLocationTranspose;
    //location of motor is logged via its index in vector and the propLocatiion vector
    vector<std::unique_ptr<motor>> motors;
    //droneBattery
    std::unique_ptr<battery> droneBattery;

    std::unique_ptr<quaternionVehicle> pose;
    indexCoordinates index;
    //helper Functions
    void rotationHelper(Quaternion& q);

    void resetHelper();

    protected:

    public:
    droneBody();
    ~droneBody();
    std::string droneConfig;
    //droneBody(const droneBody& drone) = delete;
    void updateState() override; 
    void init(string& motorConfig ,string& batteryConfig);
    //Set Square allows the creation of a rectagular prop profile.
    //positive x = front , positive y = right, positive Z = top
    void setSquare(float x , float y , float propellerMOI);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(array<float,3> offset);

    void motorMoment();

    inline void thrustRequest(vector<float>& thrust){
        thrustRequest = thrust;
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
    std::unique_ptr<PIDController> PIDX;
    std::unique_ptr<PIDController> PIDY;
    std::unique_ptr<PIDController> PIDZ;

    protected:
    public:
    unique_ptr<droneBody> body;
    droneControl();
    //initialization out of constructor due to out of order calls in unreal engine.
    
    void init(std::string& motorConfig, std::string& batteryConfig); 
    void initpidControl();
    std::array<float , 3> pidControl(float x , float y, float z);
    void setpidControl(float xTarget , float yTarget , float zTarget);
};
} //SimCore


#endif