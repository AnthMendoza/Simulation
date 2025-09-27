#ifndef WINDGENERATOR_H
#define WINDGENERATOR_H 

#include <random>
#include <memory>
#include "../utility/time_manager.h"

namespace SimCore{
class turbulence{
public:
    turbulence(float timeConstant = 5.0f, float mean = 0.0f, float stddev = 1.0f, float triggerInterval = 0.01f)
        : timeConstant(timeConstant), mean(mean), stddev(stddev), wind(mean) {
        std::random_device rd;
        rng = std::mt19937(rd());
        dist = std::normal_distribution<float>(0.0, 1.0);
        manager = std::make_shared<timeManager>(triggerInterval);
    }

    float getNext(float currentTimeSeconds) {
        if (!manager->shouldTrigger(currentTimeSeconds)) {
            return wind;
        }
        
        float dt = manager->getActualDeltaTime();
        if(dt <= 0) dt = triggerInterval;
        
        float alpha = 1.0f / timeConstant;
        float noise = dist(rng);
        wind += alpha * (mean - wind) * dt + stddev * std::sqrt(dt) * noise;
        return wind;
    }

    void reset() {
        wind = mean;
    }

    void setTimeConstant(float tc) {
        timeConstant = tc;
    }

    void setMean(float m) {
        mean = m;
    }

    void setStdDev(float s) {
        stddev = s;
    }

private:
    float timeConstant;
    float mean;
    float stddev;
    float wind;
    float triggerInterval;

    std::shared_ptr<timeManager> manager;
    std::mt19937 rng;
    std::normal_distribution<float> dist;
};
}


#endif
