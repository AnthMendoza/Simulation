#pragma once
#include <array>
#include <stdint.h>
#include <base/base.h>
#include <gps.h>

namespace SimCore{
//simple due to single IMU and gyro. more complex systems have mulitple sensors.
struct simpleSensorPacket{
    struct IMUAccelerometer{
        std::string identifier = "accelerometer";
        units::vec3 data;
        float temperature;
        float timestamp; //S
    } accelerometer;
    struct IMUGyro{
        std::string identifier = "gyro";
        units::vec3 rotationRate;
        float temperature;
        float timestamp; //S
    } gyro;
    struct GPS {
        std::string identifier = "GNSS";
        units::vec3 relativePosition;
        avionics::sensor::gps_coordinate coordinate;
        units::vec3 velocity;
        float hdop;
        float vdop;
        uint8_t satellites;
        float timestamp; //S
    } gps;
};

}