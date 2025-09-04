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
#include "../utility/utility.h"
#include "../control/dronePIDControl.h"
#include "../control/droneControllerBase.h"

using namespace std;
namespace SimCore{

class droneBody :  public Vehicle{
    private:
    void motorThrust(float motorRPM);

    vector<float> thrustRequestVect;
    int transposeCalls;
    //index vectors
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
    float totalThrustLimit = 0;
    
    droneBody(std::unique_ptr<battery> bat, std::unique_ptr<droneControllerBase> _controller)
        : droneBattery(std::move(bat)), controller(std::move(_controller)) 
    {
        if (!controller) throw std::runtime_error("Controller is nullptr when drone is constructed.");
        if (!droneBattery)throw std::runtime_error("Battery is nullptr when drone is constructed.");
    }

    ~droneBody();
    droneBody(const droneBody& other);
    string droneConfig;
    unique_ptr<droneControllerBase> controller;
    //droneBody(const droneBody& drone) = delete;
    void updateState() override; 
    void initDrone(string& droneBody);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(array<float,3> offset);

    void rotateLocalEntities(Quaternion& quant) override;

    void addMotorsAndProps(std::pair<std::vector<std::unique_ptr<motor>>,std::vector<std::unique_ptr<propeller>>>& motorPropPairs){
        for(int i = 0 ; i < motorPropPairs.first.size() ; i++){
            motors.push_back(std::move(motorPropPairs.first[i]));
            propellers.push_back(std::move(motorPropPairs.second[i]));
        }
        allocatorHelper();
        dynoSystem();
        controller->updateCalculatedValues(*this);
    }

    // Manual set for a new controller
    void setController(std::unique_ptr<droneControllerBase> newController) {
        controller = std::move(newController);
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
        if(controller == nullptr) throw std::runtime_error("Controller is a nullptr in updateController.\n");
        return controller->update(getPositionVector(),getPose(),getVelocityVector(),getTime());
    }

    void motorMoment();


    inline void thrustRequest(vector<float>& thrust){
        thrustRequestVect = thrust;
    }
    /**
    * @brief Cycles through updating controller, allocator, motors, propellers, battery, and droneBody. 
    * This method is just a connecting bridge for objects within the droneBody.
    */
    void transposedProps(Quaternion& quant);

    inline battery* getBattery(){
        battery* bat = dynamic_cast<battery*> (droneBattery.get()); 
        return bat;
    }
    inline droneControllerBase* getController(){
        return controller.get();
    }
    inline poseState getPose(){
        poseState stateVectors =  pose->getPose();
        return stateVectors;
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
        auto moments = controller->getMoments();
        std::cout << "requested Moments (mx,my,mz): ("
                  << moments[0] << ", "
                  << moments[1] << ", "
                  << moments[2] << ")\n";
        std::cout << "Motors (rad/s):    ";
        for(int i = 0; i<motors.size();i++){
                  std::cout<< std::fixed << std::setprecision(2) << motors[i]->getCurrentAngularVelocity() << ",";
        }
        std::cout<<"\nAngular Velocity request:";
        for(int i = 0; i<motors.size();i++){
                  std::cout<< std::fixed << std::setprecision(2) <<motors[i]->getAngularVelocityRequest()<< ",";
        }
        std::cout<<"\nMotor Voltage : ";
        for(int i = 0; i<motors.size();i++){
                  std::cout<< std::fixed << std::setprecision(2) << motors[i]->getVoltage() << ",";
        }
        std::cout<<"\nThrusts :";
        float density = airDensity(Zposition);
        for(int i = 0; i<motors.size();i++){
                  std::cout<< std::fixed << std::setprecision(2) <<propellers[i]->thrustForce(density , motors[i]->getCurrentAngularVelocity())<< ",";
        }        
        std::cout<<"\nProp Direction :";
        for(int i = 0 ; i < propellers.size(); i++){
            const auto& dir = propellers[i]->directionTransposed;
            std::cout << "Propeller " << i << ": ["
                        << dir[0] << ", "
                        << dir[1] << ", "
                        << dir[2] << "]\n";
        }
        std::cout<<"\nBattery Voltage:"<< droneBattery->getBatVoltage();
        std::cout << std::flush;
    }


};


} //SimCore


#endif  