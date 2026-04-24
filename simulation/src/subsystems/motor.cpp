#include "../../include/subsystems/motor.h"
#include <util/toml.h>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <assert.h>
#include <util/yaml.h>

namespace SimCore{
motor::motor(std::string& config){
    init(config);
}
motor::motor(const motor& other){
    freeSpeedAngularVelocity = other.freeSpeedAngularVelocity;
    stallTorque = other.stallTorque;
    currentLimit = other.currentLimit;
    noLoadCurrent = other.noLoadCurrent;
    coilResistance = other.coilResistance;
    voltage = other.voltage;
    kv = other.kv;
    kt = other.kt;
    ke = other.ke;
    dampingCoeff = other.dampingCoeff;
    netTorque = other.netTorque;
    motorTorque = other.motorTorque;
    currentAngularVelocity = other.currentAngularVelocity;
    currentTorque = other.currentTorque;
    currentCurrent = other.currentCurrent;
    currentThrottle = other.currentThrottle;
    maxCurrentAvailable = other.maxCurrentAvailable;
    maxVoltage = other.maxVoltage;
    appliedVoltage = other.appliedVoltage;
    inertia = other.inertia;
    angularVeloRequest = other.angularVeloRequest;
    backEMF = other.backEMF;
    electricalPower = other.electricalPower;
    mechanicalPower = other.mechanicalPower;
    efficiency = other.efficiency;
    isEnabled = other.isEnabled;
    //using PID copy constructor 
    PID = other.PID ? std::make_unique<PIDController>(*other.PID) : nullptr;
}
motor::~motor() {
}

void motor::init(std::string& config){
    YAML::Node node = YAML::LoadFile(config);
    const auto& motor = node["motor"];
    const auto& pid   = motor["pid"];

    freeSpeedAngularVelocity = utility::getRequired<float>(motor, "freeSpeedAngularVelocity", "motor");
    stallTorque              = utility::getRequired<float>(motor, "stallTorque", "motor");
    currentLimit             = utility::getRequired<float>(motor, "currentLimit", "motor");
    noLoadCurrent            = utility::getRequired<float>(motor, "noLoadCurrent", "motor");
    coilResistance           = utility::getRequired<float>(motor, "coilResistance", "motor");
    voltage                  = utility::getRequired<float>(motor, "voltage", "motor");
    kv                       = utility::getRequired<float>(motor, "kv", "motor");
    dampingCoeff             = utility::getRequired<float>(motor, "dampingCoeff", "motor");
    maxVoltage               = utility::getRequired<float>(motor, "maxVoltage", "motor");
    inertia                  = utility::getRequired<float>(motor, "inertia", "motor");

    PID = std::make_unique<PIDController>(
        utility::getRequired<float>(pid, "kp", "motor.pid"),
        utility::getRequired<float>(pid, "ki", "motor.pid"),
        utility::getRequired<float>(pid, "kd", "motor.pid")
    );


    PID->setOutputLimits(-1,1);

    currentAngularVelocity = 0;
    ke = 60.0 / (2.0 * M_PI * kv); 
    kt = ke; 
    if (ke < 0 || kt < 0 || dampingCoeff < 0) throw std::runtime_error("Physical constants must be >= 0");
     
}
//control loop modulates voltage input
void motor::angularVeloctiyRequest(float rad_per_sec){
    angularVeloRequest = rad_per_sec;
}
//the upper limit for rotor acceleration ::adjustable. done by hand calc.

void motor::updateMotor(float timeStep, float loadTorque, float voltage) {
    setVoltage(voltage);
    // Calculate back-EMF
    backEMF = ke * currentAngularVelocity;

    float voltageAcrossCoil = appliedVoltage - backEMF;
    if(coilResistance <= 0) throw std::runtime_error("coilResistance Cannot be <= 0");
    float resistanceDC = coilResistance;
    float phaseCurrent = voltageAcrossCoil / resistanceDC;
    //if(phaseCurrent > currentLimit){
    //    phaseCurrent = currentLimit;
    //    voltageAcrossCoil = phaseCurrent * resistanceDC;
    //}
    currentCurrent = phaseCurrent;
    
    
    motorTorque = kt * phaseCurrent;
    float dampingTorque = dampingCoeff * currentAngularVelocity;
    netTorque = motorTorque - loadTorque - dampingTorque;
    
    // integrate
    if(inertia <= 0) throw std::runtime_error("inertia Cannot be <= 0");
    float angularAcceleration = netTorque / inertia;
    angularAcceleration = std::clamp(angularAcceleration ,-MAX_ANGULAR_ACCEL ,MAX_ANGULAR_ACCEL);
    currentAngularVelocity += angularAcceleration * timeStep;
    
    electricalPower = appliedVoltage * phaseCurrent;
    mechanicalPower = motorTorque * currentAngularVelocity;
    efficiency = (electricalPower > 0.01f) ? (mechanicalPower / electricalPower): 0.0f;
}

void motor::updateMotorAngularVelocity(float timeStep , float loadTorque, hardware::battery& bat, float rad_sec){
    PID->setTarget(rad_sec);
    PID->setGains(.5,.2,0); 
    angularVeloRequest = rad_sec;
    float output = PID->update(currentAngularVelocity,timeStep);
    float volt = output * std::abs(bat.getBatVoltage());

    updateMotor(timeStep,loadTorque,volt);
}




}