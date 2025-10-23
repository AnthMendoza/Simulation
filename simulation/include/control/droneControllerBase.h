#pragma once
#include <iostream>
#include <vector>
#include "../control/droneControl.h"
#include "../subsystems/propeller.h"
#include "../utility/time_manager.h"
#include "controlRequestStructs.h"
#include "stateInfo.h"
#include <variant>
#include <optional>

using namespace std;

namespace SimCore{
class controlAllocator;
class droneControllerBase {
private:
protected:
    
    std::unique_ptr<timeManager> manager;
    std::array<float,3> controlOutput;
    std::array<float,3> controlOutputVelocity;
    std::array<float,3> desiredNormal;
    std::array<float,3> currentFlightTargetNormal;
    std::array<float,3> aotVect;
    std::array<float,3> logRequestedMoments = {0,0,0};
    std::unique_ptr<controlAllocator> allocator;

    std::optional<std::vector<std::array<float,3>>> propThrustPosition;
    std::optional<std::vector<std::array<float,3>>> propThrustDirectionVector;

    std::optional<float> time;

    controlPacks::forceMoments computedForceMoments;

    
public:
    bool controlEnabled = true;
    
    droneControllerBase(float timeBetweenUpdates){
        manager = std::make_unique<timeManager>(timeBetweenUpdates);
    }
    virtual ~droneControllerBase() = default;
    droneControllerBase(const droneControllerBase& other):
        controlOutput(other.controlOutput),
        controlOutputVelocity(other.controlOutputVelocity),
        desiredNormal(other.desiredNormal),
        currentFlightTargetNormal(other.currentFlightTargetNormal),
        aotVect(other.aotVect),
        logRequestedMoments(other.logRequestedMoments),
        controlEnabled(other.controlEnabled)
    {
        if(other.manager) manager = std::make_unique<timeManager>(*other.manager);
        if(other.allocator) allocator = std::make_unique<controlAllocator>(*other.allocator);
    }

    virtual std::unique_ptr<droneControllerBase> clone() const = 0;
    //call when done values change. exmaple a change in mass.
    virtual void updateCalculatedValues() = 0;

    virtual void initController(std::string droneConfig) = 0;

    virtual controlPacks::variantPackets update(float time,stateInfo statePacket) = 0;

    virtual controlPacks::forceMoments updateWithoutAllocator(float time,stateInfo statePacket) = 0;
    
    virtual void setTargetPosition(float xTarget, float yTarget, float zTarget) = 0;
    
    inline void setTargetNormalVector(float x, float y, float z) {
        desiredNormal = normalizeVector<float>({x, y, z});
    }

    
    /*
    std::array<float,3> thrustMoment(const propeller& prop ,const motor& mot, std::array<float,3> &cogLocation ,const float& airDensity){
        std::array<float,3> calculatedMoments;
        std::array<float,3> torqueVector;
        auto propLocation = prop.getLocation();
        torqueVector[0] = propLocation[0] - cogLocation[0];
        torqueVector[1] = propLocation[1] - cogLocation[1];
        torqueVector[2] = propLocation[2] - cogLocation[2];

        float  thrust = prop.thrustForce(airDensity,mot.getCurrentAngularVelocity());

        if(isZeroVector(prop.direction)) throw runtime_error("Prop Direction is a zero vector. In thrustMoment \n");
        std::array<float,3> direction = normalizeVector(prop.direction);


        std::array<float,3> thrustVector;
        thrustVector[0] = direction[0] * thrust;
        thrustVector[1] = direction[1] * thrust;
        thrustVector[2] = direction[2] * thrust;

        vectorCrossProduct(torqueVector,thrustVector,calculatedMoments);

        logRequestedMoments = calculatedMoments;

        return calculatedMoments;
    }
    */

    void initAllocator(vector<std::array<float,3>> pos, vector<std::array<float,3>> thrustVect, vector<float >coef){
        propThrustPosition = pos;
        propThrustDirectionVector = thrustVect;
        allocator = std::make_unique<controlAllocator>(pos,thrustVect,coef);

    }

    void initAllocatorWithProps(std::vector<std::unique_ptr<propeller>>& propellers){
        vector<array<float,3>> pos; 
        vector<array<float,3>> thrustVect;
        vector<float> coef;
        for(int i = 0 ; i < propellers.size();i++){
            pos.push_back(propellers[i]->location);
            thrustVect.push_back(propellers[i]->direction);
            coef.push_back(propellers[i]->powerCoefficient * static_cast<float>(propellers[i]->rotationDirection));
        }
        initAllocator(pos,thrustVect,coef);

    }

    std::array<float,3> getLoggedRequestedMoments(){
        return logRequestedMoments;
    }

    std::optional<std::vector<std::array<float, 3>>> getPropThrustPosition() const{
        return propThrustPosition;
    }

    std::optional<std::vector<std::array<float, 3>>> getPropThrustDirectionVector() const{
        return propThrustDirectionVector;
    }
    

};

}