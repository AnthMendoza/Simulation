#include "../../include/subsystems/droneSensorSuite.h"
#include "../../include/subsystems/sensors.h"
#include "../../include/sim/toml.h"
using namespace SimCore;

droneSensorSuite::droneSensorSuite(std::string& configFile){
    toml::tomlParse accelParse;
    accelParse.parseConfig( configFile,packet.accelerometer.identifier);

    // Accelerometer
    std::shared_ptr<SimCore::accelerometer> accel = std::make_shared<SimCore::accelerometer>(
        accelParse.getFloat("frequency"),
        accelParse.getFloat("NoisePowerSpectralDensity"),
        accelParse.getFloat("bandwidth"),
        accelParse.getFloat("bias")
    );
    if (accelParse.getBool("burst") == true) {
        accel->setBurst(
            accelParse.getFloat("burstStdDeviation"),
            accelParse.getFloat("maxBurstDuration")
        );
    }
    sensorMap->insert({packet.accelerometer.identifier, accel});

    // GNSS
    toml::tomlParse gpsParse;
    gpsParse.parseConfig( configFile,packet.gps.identifier);

    std::shared_ptr<SimCore::GNSS> gps = std::make_shared<SimCore::GNSS>(
        gpsParse.getFloat("frequency"),
        gpsParse.getFloat("NoisePowerSpectralDensity"),
        gpsParse.getFloat("bandwidth"),
        gpsParse.getFloat("bias")
    );

    if (gpsParse.getBool("burst") == true) {
        gps->setBurst(
            gpsParse.getFloat("burstStdDeviation"),
            gpsParse.getFloat("maxBurstDuration")
        );
    }
    sensorMap->insert({packet.gps.identifier, gps});

    // Gyroscope
    toml::tomlParse gyroParse;
    gyroParse.parseConfig( configFile,packet.gyro.identifier);
    
    std::shared_ptr<SimCore::gyroscope> gyro = std::make_shared<SimCore::gyroscope>(
        gyroParse.getFloat("frequency"),
        gyroParse.getFloat("NoisePowerSpectralDensity"),
        gyroParse.getFloat("bandwidth"),
        gyroParse.getFloat("bias")
    );
    if (gyroParse.getBool("burst") == true) {
        gyro->setBurst(
            gyroParse.getFloat("burstStdDeviation"),
            gyroParse.getFloat("maxBurstDuration")
        );
    }
    sensorMap->insert({"gyro",gyro});

}

void droneSensorSuite::updateAccelPacket() {
    auto accelShared = accelPtr.lock();
    if (!accelShared) {
        std::string identifier = packet.accelerometer.identifier;
        auto iterator = sensorMap->find(identifier);
        if (iterator == sensorMap->end()) return;
        accelShared = std::dynamic_pointer_cast<SimCore::accelerometer>(iterator->second);
        if (!accelShared) return;

        accelPtr = accelShared; 
    }
    packet.accelerometer.data = accelShared->read();
    packet.accelerometer.timestamp = accelShared->getTimeOfSample();
}

void droneSensorSuite::updateGPSPacket() {
    auto gpsShared = gpsPtr.lock();
    if (!gpsShared) {
        std::string identifier = packet.gps.identifier;
        auto iterator = sensorMap->find(identifier);
        if (iterator == sensorMap->end()) return;

        gpsShared = std::dynamic_pointer_cast<SimCore::GNSS>(iterator->second);
        if (!gpsShared) return;

        gpsPtr = gpsShared;
    }
    packet.gps.relativePosition = gpsShared->read();
    packet.gps.timestamp = gpsShared->getTimeOfSample();
}


void droneSensorSuite::updateGyroPacket() {
    auto gyroShared = gyroPtr.lock();
    if (!gyroShared) {
        std::string identifier = packet.gyro.identifier;
        auto iterator = sensorMap->find(identifier);
        if (iterator == sensorMap->end()) return;

        gyroShared = std::dynamic_pointer_cast<SimCore::gyroscope>(iterator->second);
        if (!gyroShared) return;

        gyroPtr = gyroShared;
    }
    packet.gyro.rotationRate = gyroShared->read();
    packet.gyro.timestamp = gyroShared->getTimeOfSample();
}


void droneSensorSuite::updatePacket(){
    updateAccelPacket();
    updateGPSPacket();
    updateGyroPacket();
}


void droneSensorSuite::updateSensors(SimCore::Vehicle* vehicle){
    SimCore::sensorSuite<simpleSensorPacket>::updateSensors(vehicle);
}