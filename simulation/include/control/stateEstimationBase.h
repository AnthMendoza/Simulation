#pragma once
#include <array>
#include "../core/quaternion.h"
#include "stateInfo.h"
#include "../utility/time_manager.h"
#include "../subsystems/sensorPacket.h"

namespace SimCore{

class stateEstimationBase{
    private:
    protected:
    std::unique_ptr<timeManager> manager;
    stateInfo statePacket;
    //TimeStamp is NOT the most recent call of updateEstimation but rather the most recent estimation calculation. 
    //NOTE not all upadate calls should result in an update. It should be frequency bound.
    float timestamp;

    void estimatorPacketRefresh(){
        statePacket.position = getEstimatedPosition();
        statePacket.pose = getEstimatedPose();
        statePacket.velocity = getEstimatedVelocity();
        statePacket.absVelocity = vectorMag(statePacket.velocity);
        statePacket.timestamp =  getTimestamp();
    }

    public:
    stateEstimationBase(float timeBetweenUpdates){
        manager = std::make_unique<timeManager>(timeBetweenUpdates);
    }

    virtual ~stateEstimationBase() = default;

    stateEstimationBase(const stateEstimationBase& other)
        :statePacket(other.statePacket),
        timestamp(other.timestamp){

        manager = std::make_unique<timeManager>(other.manager->getInterval());
    }

    
    virtual void updateEstimation(float time, simpleSensorPacket packet) = 0;

    virtual void controllerData(controllerToEstimationVariant){
        
    }

    inline std::array<float,3> getEstimatedPosition() const {
        return statePacket.position;
    }
    inline poseState getEstimatedPose() const {
        return statePacket.pose;
    }
    
    inline std::array<float,3> getEstimatedVelocity() const {
        return statePacket.velocity;
    }  
    inline float getAbsEstimatedVelocity() const {
        return statePacket.absVelocity;
    } 

    float getTimestamp() const{
        return timestamp;
    }

    inline stateInfo getStateInfo(){
        estimatorPacketRefresh();
        return statePacket;
    }

};


}
