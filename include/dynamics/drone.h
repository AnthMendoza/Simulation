#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"
#include "../subsystems/motor.h"
#include "../subsystems/battery.h"
#include "../core/quaternion.h"
#include "../core/indexVectors.h"
#include "../control/PIDController.h"
#include "../subsystems/propeller.h"
#include "../control/droneControl.h"
#include "aero.h"
#include <utility>
#include <memory>
#include <string>
#include "../core/vectorMath.h"
using namespace std;
namespace SimCore{

//Gives a high level state request that is handled down stream
class droneControl{
    private:
    //high level positional control loop
    unique_ptr<PIDController> PIDX;
    unique_ptr<PIDController> PIDY;
    unique_ptr<PIDController> PIDZ;
    //mid level velocity Control.
    unique_ptr<PIDController> PIDVX;
    unique_ptr<PIDController> PIDVY;
    unique_ptr<PIDController> PIDVZ;
    //low level vehicle angle control loop
    //only a single APID is needed to control the vehicles normal axis becuase the cross product is used to find the moment axis.
    unique_ptr<PIDController> APID;
    
    /// @brief 
    /// @param estimatedPostion 
    /// @param estimatedState 
    /// @return  vector of thrusts requests for each motor. 
    protected:
    public:
    //controlOutput is the ouput of the PID controllers when pidControl is called.
    //PID output clamped -1 to 1.
    std::array<float,3> controlOutput;
    std::array<float,3> controlOutputVelocity;
    std::array<float,3> desiredNormal;
    std::array<float,3> currentFlightTargetNormal;
    std::array<float,3> aotVect;

    droneControl();
    droneControl(const droneControl& other);

    //initialization out of constructor due to out of order calls in unreal engine.
    unique_ptr<controlAllocator> allocator;
    void init(); 
    //goal is to move towards a predicive model
    void initpidControl(string droneConfig,float timeStep);
    /// @brief 
    /// @param pos m
    /// @param velo m/s
    /// @param state Direction vector
    /// @param maxAngleAOT rads
    /// @return std::pair first and second is moments about x and y respectivly. Note this is not global, x and y are realtive to the drone.
    std::array<float,3> pidControl(const std::array<float,3> pos , std::array<float,3> velo  , const std::array<float,3> state, const float& maxAngleAOT);
    void setpidControl(float xTarget , float yTarget , float zTarget);
    //feedForward function for windprediction and gravity offset.
    void forceMomentProfile();

    std::array<float,3> thrustMoment(const propeller& prop ,const motor& mot, std::array<float,3>& cogLocation ,const float& airDensity);
    /// @brief 
    /// @param axisOfRotation 
    /// @param moment n*m

    vector<float> update(const std::array<float,3>& estimatedPostion,const std::array<float,3>& estimatedState,const std::array<float,3>& estimatedVelocity ,const float mass,const float gravitationalAcceleration, float thrustLimit , float maxAngleOfAttack);
    /**
     * @return pair first is the axis of rotation the second is the pid return clamped 1 to -1.The output should be converted into a deired moment.
     */
    std::pair<std::array<float,3> , float> aot(float maxAngleAOT ,std::array<float,3> currentState);
    std::pair<std::array<float,3> , float> aotControl(std::array<float,3> desiredNormal,std::array<float,3> currentState);
    // nominally desired normal should be set to {0,0,1}. Hover right side up.
    inline void setTargetNormalVecotr(float x , float y , float z){
        //Must Specify type here
        desiredNormal = normalizeVector<float>({x,y,z});
    }

    inline void setPIDXGains(const std::tuple<float, float, float>& pid) {
        if (PIDX) PIDX->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDYGains(const std::tuple<float, float, float>& pid) {
        if (PIDY) PIDY->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDZGains(const std::tuple<float, float, float>& pid) {
        if (PIDZ) PIDZ->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDVXGains(const std::tuple<float, float, float>& pid) {
        if (PIDVX) PIDVX->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDVYGains(const std::tuple<float, float, float>& pid) {
        if (PIDVY) PIDVY->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDVZGains(const std::tuple<float, float, float>& pid) {
        if (PIDVZ) PIDVZ->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setAPIDGains(const std::tuple<float, float, float>& pid) {
        if (APID) APID->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
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
    float totalThrustLimit;
    float maxAngleAOT;
    array<float,3> cogLocation;
    //transpose Locations are rotated with the vehicle. 
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

    void dynoSystem();
    protected:
    //array<float,3>  thrustVector();
    vector<float> thrust();
    public:
    droneBody();
    ~droneBody();
    droneBody(const droneBody& other);
    string droneConfig;
    unique_ptr<droneControl> controller;
    //droneBody(const droneBody& drone) = delete;
    void updateState() override; 
    void initDrone(string& motorConfig ,string& batteryConfig , string& droneBody);
    void setSquare(float x , float y , propeller& prop , motor& mot);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(array<float,3> offset);

    // Manual set for a new controller
    inline void setController(droneControl* control){
        controller = make_unique<droneControl>(*control);
    }
    
    /// @brief Sets motors to zero rpm, torque, current, and applied volatge.
    inline void resetMotors(){
        for(auto& motor:motors){
            motor->resetMotor();
        }
    }

    /**
     * @brief Simply calls the controller and feeds in the estimated Positions from state estimation as its arguments.
     * State estimation uses the sensors associated with the vehicle base class.
     * @return Requested Thrust value from controller
     */
    inline vector<float> updateController(){
        return controller->update(getEstimatedPosition(),getState(),getEstimatedVelocity(),mass,gravitationalAcceleration,totalThrustLimit,maxAngleAOT);
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
    inline droneControl* getController(){
        return controller.get();
    }

    void setMotorHover(){
        float thrust = mass/motors.size();
        for(int i = 0 ; i < motors.size() ; i++){
            float rad_sec = propellers[i]->desiredAngularVelocity(airDensity(Zposition), thrust);
            motors[i]->setMotorAngularVelocity(rad_sec);
        }
    }

    inline void droneDisplay() const {
        static const int linesToClear = 0; // number of lines in display

        // Move cursor up to overwrite previous lines
        for (int i = 0; i < linesToClear; ++i)
            std::cout << "\x1b[1A" << "\x1b[2K";  // move up 1 line + clear line
        std::cout << "Motors (rad/s):    ";
        for(int i = 0; i<motors.size();i++){
                  std::cout<< std::fixed << std::setprecision(2) << motors[i]->getCurrentAngularVelocity() << ",";
        }
        std::cout << std::flush;
    }


};

/// @brief Converts quaternion style axis + moment to eular moments.
/// @param axisOfRotation unitless
/// @param moment n*m
/// @return array where x,y,z represet moments around the axis {1,0,0},{0,1,0},and {0,0,1}.
inline std::array<float,3> axisMomentToEularMoment(std::array<float,3> axisOfRotation, float moment ){
    if(isZeroVector(axisOfRotation)) return {0,0,0};
    axisOfRotation = normalizeVector(axisOfRotation);
    axisOfRotation[0] = axisOfRotation[0] * moment;
    axisOfRotation[1] = axisOfRotation[1] * moment;
    axisOfRotation[2] = axisOfRotation[2] * moment;
    return axisOfRotation;
}

} //SimCore


#endif  