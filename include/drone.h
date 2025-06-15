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
#include "droneControl.h"
#include <utility>
#include <memory>
#include <string>
#include "vectorMath.h"
using namespace std;
namespace SimCore{

//Gives a high level state request that is handled down stream
//Container for Drone body
class droneControl{
    private:
    //high level positional control loop
    unique_ptr<PIDController> PIDX;
    unique_ptr<PIDController> PIDY;
    unique_ptr<PIDController> PIDZ;
    //low level vehicle angle control loop
    unique_ptr<PIDController> APIDX;
    unique_ptr<PIDController> APIDY;
    protected:
    public:
    //controlOutput is the ouput of the PID controllers when pidControl is called.
    //PID output clamped -1 to 1.
    std::array<float,3> controlOutput;
    std::array<float,3> desiredNormal;
    std::array<float,3> currentFlightTargetNormal;
    std::array<float,3> aotVect;
    droneControl();
    //initialization out of constructor due to out of order calls in unreal engine.
    unique_ptr<controlAllocator> allocator;
    void init(); 
    //goal is to move towards a predicive model
    void initpidControl(string droneConfig,float timeStep);
    void pidControl(std::array<float,3> pos);
    void setpidControl(float xTarget , float yTarget , float zTarget);
    //feedForward function for windprediction and gravity offset.
    void forceMomentProfile();

    void aot();
    // nominally desired normal should be set to {0,0,1}. Hover right side up.
    inline void setTargetNormalVecotr(float x , float y , float z){
        desiredNormal = normalizeVector({x,y,z});
    }
};

class droneBody :  public Vehicle{
    private:
    void motorThrust(float motorRPM);

    vector<float> thrustRequestVect;
    //true if props are already formed
    bool propLocationsSet;
    int transposeCalls;
    //index vectors

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
    /** 
    * @brief Overwrites existing allocator object.
    * allocationHelper should be called whenever a rotor is changed. Position, thrust vector, addtional motors etc.
    */
    void allocatorHelper();
    protected:
    //array<float,3>  thrustVector();
    vector<float> thrust();
    public:
    droneBody();
    ~droneBody();
    string droneConfig;
    unique_ptr<droneControl> controller;
    //droneBody(const droneBody& drone) = delete;
    void updateState() override; 
    void init(string& motorConfig ,string& batteryConfig , string& droneBody);
    void setSquare(float x , float y , propeller prop);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(array<float,3> offset);
    /**
     * @brief Simply calls the controller and feeds in the estimated Positions from state estimation as its arguments.
     * State estimation uses the sensors associated with the vehicle base class.
     */
    inline void updateController(){
        controller->pidControl(getEstimatedPosition());
    }

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

} //SimCore


#endif  