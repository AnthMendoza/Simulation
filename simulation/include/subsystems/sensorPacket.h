#pragma once
#include <array>
#include <stdint.h>
#include "../core/quaternion.h"

namespace SimCore{
//simple due to single IMU and gyro. more complex systems have mulitple sensors.
struct simpleSensorPacket{
    struct IMUAccelerometer{
        std::string identifier = "accelerometer";
        SimCore::threeDState data;
        float temperature;
        float timestamp; //S
    } accelerometer;
    struct IMUGyro{
        std::string identifier = "gyro";
        SimCore::threeDState rotationRate;
        float temperature;
        float timestamp; //S
    } gyro;
    struct GPS {
        std::string identifier = "GNSS";
        SimCore::threeDState relativePosition;
        double latitude;
        double longitude;
        float altitude;
        SimCore::threeDState velocity;
        float hdop;
        float vdop;
        uint8_t satellites;
        float timestamp; //S
    } gps;
};

}