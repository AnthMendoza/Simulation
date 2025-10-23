#pragma once
#include "sensors.h"
#include "../dynamics/vehicle.h"
#include "sensorPacket.h"
#include <string>
#include <memory>
namespace SimCore{


class droneSensorSuite : public SimCore::sensorSuite<simpleSensorPacket>{
    private:
    //caching
    std::weak_ptr<SimCore::accelerometer> accelPtr;
    std::weak_ptr<SimCore::GNSS> gpsPtr;
    std::weak_ptr<SimCore::gyroscope> gyroPtr;

    simpleSensorPacket packet;
    void updatePacket();

    void updateAccelPacket();
    void updateGPSPacket();
    void updateGyroPacket();
    public:
    droneSensorSuite(std::string& configFile);

    void updateSensors(SimCore::Vehicle *vehicle) override;

    simpleSensorPacket getSensorData() override{
        updatePacket();
        return packet;
    }
};

}