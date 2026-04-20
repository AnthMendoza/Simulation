#pragma once
#include <array>
#include <stdint.h>
#include <base/base.h>
#include <gps.h>

namespace SimCore{
//simple due to single IMU and gyro. more complex systems have multiple sensors.
struct simpleSensorPacket{
    struct IMUAccelerometer{
        std::string identifier = "accelerometer";
        units::vec3 data;
        float temperature; //C
        uint64_t timestamp; //S
    } accelerometer;
    struct IMUGyro{
        std::string identifier = "gyro";
        units::vec3 rotationRate; //rad/s
        float temperature; //C
        uint64_t timestamp; //s
    } gyro;
    struct GPS {
        std::string identifier = "GNSS";
        units::vec3 relativePosition; //m
        avionics::sensor::gps_coordinate coordinate;
        units::vec3 velocity; //m/s
        float hdop;
        float vdop;
        uint8_t satellites;
        uint64_t timestamp; //S
    } gps;
};

}