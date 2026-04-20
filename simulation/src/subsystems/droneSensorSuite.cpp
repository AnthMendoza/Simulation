#include "../../include/subsystems/droneSensorSuite.h"
#include "../../include/subsystems/sensors.h"
#include <gps.h>
#include <util/yaml.h>
#include <yaml-cpp/yaml.h>

using namespace SimCore;

droneSensorSuite::droneSensorSuite(std::string& configFile){
    YAML::Node node = YAML::LoadFile(configFile);
    
    const auto& acc_node = node[packet.accelerometer.identifier]; 

    // Accelerometer
    std::shared_ptr<SimCore::accelerometer> accel = std::make_shared<SimCore::accelerometer>(
        utility::getRequired<float>(acc_node,"frequency","acceleration sensor"),
        utility::getRequired<float>(acc_node,"NoisePowerSpectralDensity","acceleration sensor"),
        utility::getRequired<float>(acc_node,"bandwidth","acceleration sensor"),
        utility::getRequired<float>(acc_node,"bias","acceleration sensor")
    );
    if (utility::getRequired<bool>(acc_node,"burst","acceleration sensor") == true) {
        accel->setBurst(
            utility::getRequired<float>(acc_node,"burstStdDeviation","acceleration sensor"),
            utility::getRequired<float>(acc_node,"maxBurstDuration","acceleration sensor")
        );
    }
    sensorMap->insert({packet.accelerometer.identifier, accel});

    // GNSS
    const auto& gps_node = node[packet.gps.identifier];

    avionics::sensor::gps_coordinate gpsCord{
        utility::getRequired<double>(gps_node,"lat_deg","gps sensor"),
        utility::getRequired<double>(gps_node,"long_deg","gps sensor"),
        utility::getRequired<double>(gps_node,"alt_m","gps sensor")
    };

    std::shared_ptr<SimCore::GNSS> gps = std::make_shared<SimCore::GNSS>(
        utility::getRequired<float>(gps_node,"frequency","gps sensor"),
        utility::getRequired<float>(gps_node,"NoisePowerSpectralDensity","gps sensor"),
        utility::getRequired<float>(gps_node,"bandwidth","gps sensor"),
        utility::getRequired<float>(gps_node,"bias","gps sensor"),
        utility::getRequired<float>(gps_node,"velocityNoisePowerSpectralDensity","gps sensor"),
        utility::getRequired<float>(gps_node,"velocityBandwidth","gps sensor"),
        utility::getRequired<float>(gps_node,"velocityBias","gps sensor"),
        gpsCord
    );

    if (utility::getRequired<bool>(gps_node,"burst","gps sensor") == true) {
        gps->setBurst(
            utility::getRequired<float>(gps_node,"burstStdDeviation","gps sensor"),
            utility::getRequired<float>(gps_node,"maxBurstDuration","gps sensor")
        );
    }
    sensorMap->insert({packet.gps.identifier, gps});

    // Gyroscope
    const auto& gyro_node = node[packet.gyro.identifier];
    
    std::shared_ptr<SimCore::gyroscope> gyro = std::make_shared<SimCore::gyroscope>(
        utility::getRequired<float>(gyro_node,"frequency","gyro sensor"),
        utility::getRequired<float>(gyro_node,"NoisePowerSpectralDensity","gyro sensor"),
        utility::getRequired<float>(gyro_node,"bandwidth","gyro sensor"),
        utility::getRequired<float>(gyro_node,"bias","gyro sensor")
    );

    if (utility::getRequired<bool>(gyro_node,"burst","gyro sensor") == true) {
        gyro->setBurst(
            utility::getRequired<float>(gyro_node,"burstStdDeviation","gyro sensor"),
            utility::getRequired<float>(gyro_node,"maxBurstDuration","gyro sensor")
        );
    }
    sensorMap->insert({"gyro",gyro});

}

void droneSensorSuite::updateAccelPacket() {
    auto accelShared = accelPtr.lock();
    if (!accelShared) {
        //cache location to prevent recall of find()
        std::string identifier = packet.accelerometer.identifier;
        auto iterator = sensorMap->find(identifier);
        if (iterator == sensorMap->end()) return;
        accelShared = std::dynamic_pointer_cast<SimCore::accelerometer>(iterator->second);
        if (!accelShared) return;

        accelPtr = accelShared; 
    }
    packet.accelerometer.data = accelShared->read();
    packet.accelerometer.timestamp = accelShared->getTimeOfSampleUS();
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

    packet.gps.coordinate = gpsShared->readGNSS();
    packet.gps.timestamp = gpsShared->getTimeOfSampleUS();
    packet.gps.velocity = gpsShared->readGNSSVelocity();
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
    packet.gyro.timestamp = gyroShared->getTimeOfSampleUS();
}


void droneSensorSuite::updatePacket(){
    updateAccelPacket();
    updateGPSPacket();
    updateGyroPacket();
}


void droneSensorSuite::updateSensors(SimCore::Vehicle* vehicle){
    SimCore::sensorSuite<simpleSensorPacket>::updateSensors(vehicle);
}