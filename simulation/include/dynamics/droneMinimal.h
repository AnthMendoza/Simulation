#ifndef DRONEMINIMAL_H
#define DRONEMINIMAL_H
#include "vehicle.h"
#include "../subsystems/motor.h"
#include "../subsystems/battery.h"
#include "../subsystems/propeller.h"
#include "aero.h"
#include <utility>
#include <memory>
#include <string>
#include <algorithm>
#include <sstream>
#include <optional>
#include <utility>
#include <util/utility.h>
#include <base/base.h>
#include "../subsystems/cameraBase.h"
#include "../sim/sim_to_flight.h"
#include <actual_state.h>
#include <sensor_field.h>




using namespace std;
namespace SimCore{

class drone :  public Vehicle{
    private:
    int transposeCalls;
    //index vectors
    units::vec3 cogLocation;
    //transpose Locations are rotated with the vehicle. 
    units::vec3 cogLocationTranspose;
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

    protected:
    //array<units::scalar,3>  thrustVector();
    vector<units::scalar> thrust();

    std::optional<cameraBase*> camera;

    public:
    
    drone(){

    }

    ~drone();

    drone(const drone& other);

    template<typename T, typename... Args>
    void set_battery(Args&&... args){
        static_assert(std::is_base_of_v<battery,T>);
 
        droneBattery = std::make_unique<T>(std::forward<Args>(args)...);
    }

    virtual std::unique_ptr<Vehicle> clone() const override{
        return std::make_unique<drone>(*this);
    }

    string droneConfig;
    //drone(const drone& drone) = delete;
    void updateState(units::scalar time,std::optional<controlPacks::variantPackets> controlInput= nullopt) override; 
    void initDrone(string& drone);
    //sets center of gravity as an offset relative to the center defined by propLocations
    //positive x = front , positive y = right, positive Z = top
    void offsetCOG(units::vec3 offset);

    void rotateLocalEntities(const Quaternion& quant) override;


    void addMotorsAndProps(std::pair<std::vector<std::unique_ptr<motor>>,std::vector<std::unique_ptr<propeller>>>& motorPropPairs){
        for(int i = 0 ; i < motorPropPairs.first.size() ; i++){
            motors.push_back(std::move(motorPropPairs.first[i]));
            propellers.push_back(std::move(motorPropPairs.second[i]));
        }
    }

    /// @brief Sets motors to zero rpm, torque, current, and applied volatge.
    inline void resetMotors(){
        for(auto& motor:motors){
            motor->resetMotor();
        }
    }

    vector<units::scalar> thrustRequestVect;

    inline void thrustRequest(vector<units::scalar>& thrust){
        thrustRequestVect = thrust;
    }
    /**
    * @brief Cycles through updating controller, allocator, motors, propellers, battery, and drone. 
    * This method is just a connecting bridge for objects within the drone.
    */
    void transposeEntities(const Quaternion& quant);

    inline battery* getBattery(){
        battery* bat = dynamic_cast<battery*> (droneBattery.get()); 
        return bat;
    }

    inline poseState getPose(){
        return pose->getPose();
    }

    

    using simFlight = avionics::sim_to_flight<avionics::sensor::SensorField,avionics::sensor::actual_state>;

    avionics::sensor::SensorField sensorField;

    simFlight* bridge;

    void setSimPublish(simFlight& translationBridge){
        bridge = &translationBridge;
    }


    void setEntitiesPose(const poseState& pose) override;

    void publish_simulation() override;

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
        units::scalar density = airDensity(Zposition);
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

        if(camera.has_value()){
            auto cameraPosition = camera.value()->getTransposePosition();
            buffer << "Camera position :"<< cameraPosition[0] << "," <<cameraPosition[1] << "," << cameraPosition[2] <<"\n";
            auto cameraPose = camera.value()->getTransposeState();
            auto cameraDir = cameraPose.getPose().dirVector;
            buffer << "Camera position :"<< cameraDir[0] << "," <<cameraDir[1] << "," << cameraDir[2] <<"\n";
        }

        std::string droneString = buffer.str();
        std::string vehicleString = Vehicle::display();


        
        return droneString + vehicleString;
    }

};


} //SimCore


#endif  