#pragma once
#include <iostream>
#include <vector>
#include "../dynamics/drone.h"
#include "../control/droneControl.h"
#include "../utility/time_manager.h"

using namespace std;

namespace SimCore{
class droneBody;
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
    std::vector<float> computedThrust;
    std::array<float,3> moments;
    std::unique_ptr<controlAllocator> allocator;
    
public:
    bool controlEnabled = true;
    
    droneControllerBase(float updateFrequency){
        manager = std::make_unique<timeManager>(updateFrequency);
    }
    virtual ~droneControllerBase() = default;
    droneControllerBase(const droneControllerBase& other):
        controlOutput(other.controlOutput),
        controlOutputVelocity(other.controlOutputVelocity),
        desiredNormal(other.desiredNormal),
        currentFlightTargetNormal(other.currentFlightTargetNormal),
        aotVect(other.aotVect),
        computedThrust(other.computedThrust),
        moments(other.moments),
        controlEnabled(other.controlEnabled)
    {
        if(other.manager) manager = std::make_unique<timeManager>(*other.manager);
        if(other.allocator) allocator = std::make_unique<controlAllocator>(*other.allocator);
    }

    virtual std::unique_ptr<droneControllerBase> clone() const = 0;
    //call when done values change. exmaple a change in mass.
    virtual void updateCalculatedValues(const droneBody& drone) = 0;

    virtual void initController(std::string droneConfig,const droneBody& drone) = 0;
    virtual std::vector<float> update(
        const std::array<float,3>& estimatedPosition,
        poseState state,
        const std::array<float,3>& estimatedVelocity,
        float time
    ) = 0;
    
    virtual void setTargetPosition(float xTarget, float yTarget, float zTarget) = 0;
    
    inline void setTargetNormalVector(float x, float y, float z) {
        desiredNormal = normalizeVector<float>({x, y, z});
    }
    

    std::array<float,3> thrustMoment(const propeller& prop ,const motor& mot, std::array<float,3> &cogLocation ,const float& airDensity){
        std::array<float,3> moments;
        std::array<float,3> torqueVector;
        torqueVector[0] = prop.location[0] - cogLocation[0];
        torqueVector[1] = prop.location[1] - cogLocation[1];
        torqueVector[2] = prop.location[2] - cogLocation[2];

        float  thrust = prop.thrustForce(airDensity,mot.getCurrentAngularVelocity());

        if(isZeroVector(prop.direction)) throw runtime_error("Prop Direction is a zero vector. In thrustMoment \n");
        std::array<float,3> direction = normalizeVector(prop.direction);


        std::array<float,3> thrustVector;
        thrustVector[0] = direction[0] * thrust;
        thrustVector[1] = direction[1] * thrust;
        thrustVector[2] = direction[2] * thrust;

        vectorCrossProduct(torqueVector,thrustVector,moments);
        

        return moments;
    }
    

    void initAllocator(vector<array<float,3>> pos, vector<array<float,3>> thrustVect, vector<float >coef){
        allocator = std::make_unique<controlAllocator>(pos,thrustVect,coef);
    }

    std::array<float,3> getMoments(){
        return moments;
    }

};

}