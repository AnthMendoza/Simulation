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
    /// @brief 
    /// @param estimatedPostion 
    /// @param estimatedState 
    /// @return  vector of thrusts requests for each motor. 
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

    vector<float> update(std::array<float,3> estimatedPostion,std::array<float,3> estimatedState,std::array<float,3> estimatedVelocity, float mass, float gravitationalAcceleration);

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
    void setSquare(float x , float y , propeller& prop , motor& mot);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(array<float,3> offset);
    /**
     * @brief Simply calls the controller and feeds in the estimated Positions from state estimation as its arguments.
     * State estimation uses the sensors associated with the vehicle base class.
     * @return Requested Thrust value from controller
     */
    inline vector<float> updateController(){
        return controller->update(getEstimatedPosition(),getEstimatedRotation(),getEstimatedVelocity(),mass,gravitationalAcceleration);
    }

    void motorMoment();

    inline void thrustRequest(vector<float>& thrust){
        thrustRequestVect = thrust;
    }
    /**
    * @brief Cycles through updating controller, allocator, motors, propellers, battery, and droneBody. 
    * This method is just a connecting bridge for objects within the droneBody.
    */
    void transposedProps();

    inline battery* getBattery(){
        battery* bat = dynamic_cast<battery*> (droneBattery.get()); 
        return bat;
    }
    inline void droneDisplay() const {
        static const int linesToClear = 1; // number of lines in display

        // Move cursor up to overwrite previous lines
        for (int i = 0; i < linesToClear; ++i)
            std::cout << "\x1b[1A" << "\x1b[2K";  // move up 1 line + clear line
        std::cout << "Motors (rad/s):    ";
        for(int i = 0; i<motors.size();i++){
                  std::cout<< std::fixed << std::setprecision(2) << motors[0]->getCurrentAngularVelocity() << ",";
        }
        std::cout << std::flush;
    }


};

} //SimCore


#endif  