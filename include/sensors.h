#ifndef SENSORS_H 
#define SENSORS_H


#pragma once

#include <random>
#include "vectorMath.h"
namespace SimCore{
class Vehicle;
class sensor{
    private:
    float mean;
    float standardDeviation;
    float lowerBound;
    float upperBound;
    float currentBustValue;
    bool isClamped = false;
    bool burst = false;
    float busrtStdDev;
    float maxBurstDur;
    float burstDuration;
    float lastBurst;
    float currentBurstMagnitude = 0;
    protected:
    float sampleFrequency;
    //Time of last sample
    float lastSample;

    std::random_device seed; 
    std::mt19937 gen{seed()};

    float noise();
    float linearInterpolation();
    float burstNoise(float currentTime);
    void clamp(float &input);
    public:
    float hz;
    sensor(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias);
    virtual ~sensor() =  default;
    float applyNoise(float realValue , float currentTime);
    void setClamp(float low , float high);
    void setBurst(float busrtStdDeviation , float maxBurstDuration);
    virtual void sample(Vehicle *vehicle);
    virtual std::array<float,3> read(); 

};


class sensorSuite{
    protected:
    std::unique_ptr<std::unordered_map<std::string, std::unique_ptr<sensor>>> sensorMap;
    public: 
    sensorSuite();
    void updateSensors(Vehicle *vehicle);
};


class GNSS : public sensor{
    private:
    std::array<float,3> gpsPosition;
    std::array<float,3> velocity;
    std::array<float,3> lastPosition;
    public:
    GNSS(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias);
    void sample(Vehicle *vehicle) override; 
    std::array<float,3>  read() override;
    inline std::array<float,3> getVelocity(){
        return velocity;
    }
    inline void setGNSSVelocity(std::array<float,3> velo){
        velocity = velo;
    }
};

class accelerometer : public sensor{
    private:
    std::array<float,3> accel;
    public:
    accelerometer(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias);
    void sample(Vehicle *vehicle) override;
    std::array<float,3>  read() override;

};


class gyroscope : public sensor{
    private:
    std::array<float,3> rotationVector;
    public:
    gyroscope(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias);
    void sample(Vehicle *vehicle) override;
    std::array<float,3>  read() override;

};


//class radar : public sensor{
//    private:
//    std::array<float,3> sensorOrigin;
//    std::vector<std::vector<std::array<float,3>>> points;
//    std::array<float,3> vehicleVector;
//    float quantization = .05;
//    const int rows = 16;           // vertical resolution
//    const int cols = 64;           // horizontal resolution
//    const float verticalFOV = 20;  // degrees
//    const float horizontalFOV = 90; // degrees
//    const float maxRange = 10.0f;   // meters
//    protected:
//
//    public:
//    void sample(Vehicle *Vehicle) override;
//        
//};



class stateEstimation: public sensorSuite{
    private:
    std::array<float,3> position;
    std::array<float,3> velocity;
    std::array<float,3> rotation;
    //relative positoin calculated by the most recent GPS position
    float alpha;
    bool firstGPSSample = true;
    float lowPassFilter(float newData,float prevData);
    protected:

    public:
    stateEstimation();
    //Update Estimated positions based on the most recent velocity and acceleration.
    //Update will run at simulation clock speed unlike sensors which are tied to hardware spec sample rates.
    void gpsUpdate();
    
    void updateEstimation(float timeStep);

    inline std::array<float,3> getEstimatedPosition(){
        return position;
    }
    //rotation estimation is returned as a vector pointing in the direction of the vehicles state.
    //Currently no logic in rotation estimation. this is simply a noisy model of the current vector state.
    // Simulated drift should be introduced. This also allows us to correct bias.
    
    inline std::array<float,3> getEstimatedRotation(){
        return sensorMap->find("gyro")->second->read();
    }
    
    inline std::array<float,3> getEstimatedVelocity(){
        return velocity;
    }  
    inline float getAbsEstimatedVelocity(){
        return vectorMag(velocity);
    }  
};

}
#endif
