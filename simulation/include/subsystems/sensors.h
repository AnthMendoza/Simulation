#ifndef SENSORS_H 
#define SENSORS_H


#pragma once

#include <random>
#include <optional>
#include <base/base.h>
#include <gps.h>

namespace SimCore{
class Vehicle;
class sensor{
    private:
    units::scalar mean;
    units::scalar standardDeviation;
    units::scalar lowerBound;
    units::scalar upperBound;
    units::scalar currentBustValue;
    bool isClamped = false;
    bool burst = false;
    units::scalar busrtStdDev;
    units::scalar maxBurstDur;
    units::scalar burstDuration;
    units::scalar lastBurst;
    units::scalar currentBurstMagnitude = 0;
    protected:
    units::scalar sampleFrequency;
    //Time of last sample
    units::scalar lastSample = 0;
    //Random are not copyable 
    //thus seed and gen{seed()} delete the constructor 
    std::random_device seed; 
    std::mt19937 gen{seed()};

    units::scalar noise();
    units::scalar linearInterpolation();
    units::scalar burstNoise(units::scalar currentTime);
    void clamp(units::scalar &input);
    public:
    units::scalar hz;
    sensor(units::scalar frequency , units::scalar NoisePowerSpectralDensity , units::scalar bandwidth, units::scalar bias);
    sensor(const sensor& other);
    virtual std::unique_ptr<sensor> clone() const = 0;
    virtual ~sensor() =  default;
    units::scalar applyNoise(units::scalar realValue , units::scalar currentTime);
    void setClamp(units::scalar low , units::scalar high);
    void setBurst(units::scalar busrtStdDeviation , units::scalar maxBurstDuration);
    virtual void sample(Vehicle *vehicle) = 0;
    virtual units::vec3 read() = 0; 

    inline const units::scalar getTimeOfSample() const {
        return lastSample;
    }

    inline const uint64_t getTimeOfSampleUS() const{
        return core::time::s_to_us<units::scalar,uint64_t>(lastSample);
    }

};

template<typename sensorPacketType>
class sensorSuite{
    private:
    public:
    std::unique_ptr<std::unordered_map<std::string, std::shared_ptr<sensor>>> sensorMap;

    sensorSuite(){
    sensorMap = std::make_unique<std::unordered_map<std::string, std::shared_ptr<sensor>>>();
    }

    sensorSuite(const sensorSuite& other) {
        sensorMap = std::make_unique<std::unordered_map<std::string, std::shared_ptr<sensor>>>();
        for (const auto& [key, sensorPtr] : *other.sensorMap) {
            if (sensorPtr) {
                (*sensorMap)[key] = sensorPtr->clone(); 
            }
        }   
    }
    virtual std::unique_ptr<sensorSuite<sensorPacketType>> clone() const = 0;
    virtual ~sensorSuite() = default;
    //Each sensor base class has a limiter. Only allowing for samples if the frequency allows.
    //Call at simulation frequency or at the frequency of the fastest sensor.
    //cycles through *sensorMap and calls on sample method of each sensor for potental update of data sample.
    virtual void updateSensors(Vehicle *vehicle){
        //keyPtrSensors is a std::pair first is key second is unique_ptr.
        for (const auto& keyPtrSensors : *sensorMap){
            keyPtrSensors.second->sample(vehicle);
        }
    }

    virtual sensorPacketType getSensorData() = 0;
};


#include <GeographicLib/LocalCartesian.hpp>

class GNSS : public sensor{
    private:
    units::vec3 gpsPosition;
    units::vec3 lastPosition;
    avionics::sensor::gps_coordinate gpsCordOrigin;
    avionics::sensor::gps_coordinate gpsCordCurrent;
    GeographicLib::LocalCartesian geographicCartesian;

    class GNSSVelocity: public sensor{
        units::vec3 gpsVelocity;
        public:
        GNSSVelocity(
            units::scalar frequency,
            units::scalar NoisePowerSpectralDensity,
            units::scalar bandwidth,
            units::scalar bias
        );

        std::unique_ptr<sensor> clone() const override {
            return std::make_unique<GNSSVelocity>(*this);
        }
        void sample(Vehicle *vehicle) override; 

        units::vec3 read() override;
    };
    std::unique_ptr<GNSSVelocity> velocitySensor;

    public:
    GNSS(units::scalar frequency,
        units::scalar NoisePowerSpectralDensity,
        units::scalar bandwidth,
        units::scalar bias,
        units::scalar velocityNoisePowerSpectralDensity,
        units::scalar velocityBandwidth,
        units::scalar velocityBias,
        avionics::sensor::gps_coordinate gps
    );

    GNSS(const GNSS& other);

    std::unique_ptr<sensor> clone() const override {

        return std::make_unique<GNSS>(GNSS(*this));
    }

    void sample(Vehicle *vehicle) override; 
    units::vec3 read() override;
    avionics::sensor::gps_coordinate readGNSS();
    units::vec3 readGNSSVelocity();
};

class accelerometer : public sensor{
    private:
    units::vec3 accel;
    public:
    accelerometer(units::scalar frequency , units::scalar NoisePowerSpectralDensity , units::scalar bandwidth, units::scalar bias);
    std::unique_ptr<sensor> clone() const override {
        return std::make_unique<accelerometer>(*this);
    }
    void sample(Vehicle *vehicle) override;
    units::vec3  read() override;

};


class gyroscope : public sensor{
    private:
    //rotation around x y z axis realitve to the drone vehicle. 
    units::vec3 rotationVector;
    poseState lastPose;
    public:
    gyroscope(units::scalar frequency , units::scalar NoisePowerSpectralDensity , units::scalar bandwidth, units::scalar bias);
    std::unique_ptr<sensor> clone() const override {
        return std::make_unique<gyroscope>(*this);
    }
    void sample(Vehicle *vehicle) override;
    units::vec3  read() override;

};


//class radar : public sensor{
//    private:
//    units::vec3 sensorOrigin;
//    std::vector<std::vector<units::vec3>> points;
//    units::vec3 vehicleVector;
//    units::scalar quantization = .05;
//    const int rows = 16;           // vertical resolution
//    const int cols = 64;           // horizontal resolution
//    const units::scalar verticalFOV = 20;  // degrees
//    const units::scalar horizontalFOV = 90; // degrees
//    const units::scalar maxRange = 10.0f;   // meters
//    protected:
//
//    public:
//    void sample(Vehicle *Vehicle) override;
//        
//};


}
#endif
