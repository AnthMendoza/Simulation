#ifndef TIMEMANAGER_H
#define TIMEMANAGER_H
namespace SimCore{
class timeManager {
private:
    //actual delta T is used incase timestep and time interval are not evenly divisable 
    float triggerInterval;
    float actualDeltaTime;
    float lastTriggerTime;
    bool firstCall;
    float EPSILON = 1E-6;

public:
    timeManager(float triggerIntervalSeconds = 0.01f);
    //copy constructor and asignement reset becauase new object may have diffrent starting time
    timeManager(const timeManager& other);

    timeManager& operator=(const timeManager& other);

    bool shouldTrigger(float currentSimTime);

    bool shouldTrigger(float currentSimTime, float customIntervalSeconds);
    

    void setInterval(float intervalSeconds);
    

    float getActualDeltaTime() const;
    

    float getInterval() const;
    

    float getFrequency() const;
    

    float getActualFrequency() const;
    

    void reset(float currentSimTime = 0.0f);
    

    float getLastTriggerTime() const;
};
}
#endif