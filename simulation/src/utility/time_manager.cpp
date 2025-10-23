#include "../../include/utility/time_manager.h"
namespace SimCore{
timeManager::timeManager(float triggerIntervalSeconds) 
    : triggerInterval(triggerIntervalSeconds), 
      actualDeltaTime(triggerIntervalSeconds),
      lastTriggerTime(0.0f),
      firstCall(true) {
}

timeManager::timeManager(const timeManager& other)
    : triggerInterval(other.triggerInterval),
      actualDeltaTime(other.triggerInterval),
      lastTriggerTime(0.0f),
      firstCall(true) {
}

timeManager& timeManager::operator=(const timeManager& other) {
    if (this != &other) {
        triggerInterval = other.triggerInterval;
        actualDeltaTime = other.triggerInterval;
        lastTriggerTime  = 0.0f;
        firstCall   = true;
    }
    return *this;
}


bool timeManager::shouldTrigger(float currentSimTime){
    if(triggerInterval <= 0.0f) return true;

    if(currentSimTime < lastTriggerTime){
        lastTriggerTime = currentSimTime;
    }

    float elapsed = currentSimTime - lastTriggerTime;
    
    if (firstCall || elapsed >= (triggerInterval - EPSILON)) {
        actualDeltaTime = elapsed;
        if(firstCall) actualDeltaTime = triggerInterval;
        lastTriggerTime = currentSimTime;
        firstCall = false;
        return true;
    }
    
    return false;
}

bool timeManager::shouldTrigger(float currentSimTime, float customIntervalSeconds){
    if(triggerInterval <= 0.0f) return true;
    if(currentSimTime < lastTriggerTime){
        lastTriggerTime = currentSimTime;
    }

    float elapsed = currentSimTime - lastTriggerTime;
    
    if (elapsed >= customIntervalSeconds) {
        actualDeltaTime = elapsed;
        lastTriggerTime = currentSimTime;
        return true;
    }
    
    return false;
}

void timeManager::setInterval(float intervalSeconds) {
    triggerInterval = intervalSeconds;
}

float timeManager::getActualDeltaTime() const {
    return actualDeltaTime;
}

float timeManager::getInterval() const {
    return triggerInterval;
}

float timeManager::getFrequency() const {
    if(triggerInterval <= 0){
        return 0;
    }
    return 1.0f / triggerInterval;
}

float timeManager::getActualFrequency() const {
    return actualDeltaTime > 0.0f ? 1.0f / actualDeltaTime : 0.0f;
}

void timeManager::reset(float currentSimTime) {
    lastTriggerTime = currentSimTime;
    actualDeltaTime = triggerInterval;
    firstCall = true;
}

float timeManager::getLastTriggerTime() const {
    return lastTriggerTime;
}
}