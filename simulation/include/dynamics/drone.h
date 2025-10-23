#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"
#include "../subsystems/motor.h"
#include "../subsystems/battery.h"
#include "../core/quaternion.h"
#include "../core/indexVectors.h"
#include "../subsystems/propeller.h"
#include "aero.h"
#include <utility>
#include <memory>
#include <string>
#include <algorithm>
#include <sstream>
#include "../core/vectorMath.h"
#include "../utility/utility.h"
#include "../core/droneState.h"
#include "../control/droneControllerBase.h"
#define USE_STATE_ESTIMATED_VALUES

using namespace std;
namespace SimCore{

class droneBody :  public Vehicle{
    private:
    vector<float> thrustRequestVect;
    int transposeCalls;
    //index vectors
    std::array<float,3> cogLocation;
    //transpose Locations are rotated with the vehicle. 
    std::array<float,3> cogLocationTranspose;
    //location of motor is logged via its index in vector and the propLocatiion vector
    vector<unique_ptr<motor>> motors;
    vector<unique_ptr<propeller>> propellers;
    //droneBattery
    unique_ptr<battery> droneBattery;

    indexCoordinates index;

    void dataLog();

    //helper Functions
    void rotationHelper(const Quaternion& q);

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
    float totalThrustLimit = 0;
    
    droneBody(std::unique_ptr<battery> bat)
        : droneBattery(std::move(bat))
    {
        if (!droneBattery)throw std::runtime_error("Battery is nullptr when drone is constructed.");
    }

    ~droneBody();
    droneBody(const droneBody& other);
    string droneConfig;
    //droneBody(const droneBody& drone) = delete;
    void updateState(float time,std::optional<controlPacks::variantPackets> controlInput= nullopt) override; 
    void initDrone(string& droneBody);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(std::array<float,3> offset);

    void rotateLocalEntities(const Quaternion& quant) override;


    void addMotorsAndProps(std::pair<std::vector<std::unique_ptr<motor>>,std::vector<std::unique_ptr<propeller>>>& motorPropPairs){
        for(int i = 0 ; i < motorPropPairs.first.size() ; i++){
            motors.push_back(std::move(motorPropPairs.first[i]));
            propellers.push_back(std::move(motorPropPairs.second[i]));
        }
        dynoSystem();
    }

    /// @brief Sets motors to zero rpm, torque, current, and applied volatge.
    inline void resetMotors(){
        for(auto& motor:motors){
            motor->resetMotor();
        }
    }


    inline void thrustRequest(vector<float>& thrust){
        thrustRequestVect = thrust;
    }
    /**
    * @brief Cycles through updating controller, allocator, motors, propellers, battery, and droneBody. 
    * This method is just a connecting bridge for objects within the droneBody.
    */
    void transposedProps(const Quaternion& quant);

    inline battery* getBattery(){
        battery* bat = dynamic_cast<battery*> (droneBattery.get()); 
        return bat;
    }

    inline poseState getPose(){
        return pose->getPose();
    }

    void setMotorHover(){
        float thrust = mass/motors.size();
        for(int i = 0 ; i < motors.size() ; i++){
            float rad_sec = propellers[i]->desiredAngularVelocity(airDensity(Zposition), thrust);
            motors[i]->setMotorAngularVelocity(rad_sec);
        }
    }
    void setEntitiesPose(const poseState& pose) override;


    std::string display() const override{
        std::ostringstream buffer;
        buffer << "-----Drone State-----\n";

        buffer << "Motors (rad/s):    ";
        for (auto& motor : motors) {
            buffer << std::fixed << std::setprecision(2) << motor->getCurrentAngularVelocity() << ",";
        }
        buffer << "\nAngular Velocity request:";
        for (auto& motor : motors) {
            buffer << std::fixed << std::setprecision(2) << motor->getAngularVelocityRequest() << ",";
        }
        buffer << "\nMotor Voltage : ";
        for (auto& motor : motors) {
            buffer << std::fixed << std::setprecision(2) << motor->getVoltage() << ",";
        }

        buffer << "\nThrusts :";
        float density = airDensity(Zposition);
        for (size_t i = 0; i < motors.size(); ++i) {
            buffer << std::fixed << std::setprecision(2) 
                   << propellers[i]->thrustForce(density, motors[i]->getCurrentAngularVelocity()) << ",";
        }

        buffer << "\nProp Direction/Location :\n";
        for (size_t i = 0; i < propellers.size(); ++i) {
            const auto& dir = propellers[i]->directionTransposed;
            const auto& pos = propellers[i]->locationTransposed;
            buffer << "Propeller " << i << ": ["
                   << dir[0] << ", " << dir[1] << ", " << dir[2] << "] , [" << pos[0]<<", "<< pos[1]<<", "<< pos[2] <<"]\n";    
        }
        buffer << "-----Battery State-----\n";
        buffer << "Voltage(V):" << droneBattery->getBatVoltage() << "\n";
        buffer << "Current(amps):" << droneBattery->getCurrentDraw() << "\n";
        buffer << "remaining Capacity(Wh):" << droneBattery->getRemainingEnergyWh() << "\n";
        buffer << "SOC:" << droneBattery->getSOC() << "\n";

        std::string droneString = buffer.str();
        std::string vehicleString = Vehicle::display();


        
        return droneString + vehicleString;
    }

    void setGlobals(){
        auto& state = testGlobals::actualVehicleState::state;
        state.pose = pose->getPose();
        state.velocity = getVelocityVector();
        state.position = getPositionVector();
        state.timestamp = getTime();
    }
};


} //SimCore


#endif  