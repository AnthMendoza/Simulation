#include "../../include/subsystems/motor.h"
#include "../../include/sim/toml.h"
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <assert.h>

namespace SimCore{
motor::motor(std::string& config,float timeStep){
    init(config,timeStep);
}
motor::motor(const motor& other){
    freeSpeedAngularVelocity = other.freeSpeedAngularVelocity;
    stallTorque = other.stallTorque;
    stallCurrent = other.stallCurrent;
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

void motor::init(std::string& config, float timeStep){
    toml::tomlParse mParse;
    mParse.parseConfig(config ,"motor");
    freeSpeedAngularVelocity   = mParse.getFloat("freeSpeedAngularVelocity");
    stallTorque    = mParse.getFloat("stallTorque");          
    stallCurrent   = mParse.getFloat("stallCurrent");      
    noLoadCurrent  = mParse.getFloat("noLoadCurrent");   
    coilResistance = mParse.getFloat("coilResistance");
    voltage        = mParse.getFloat("voltage");           
    kv             = mParse.getFloat("kv");                  
    kt             = mParse.getFloat("kt");
    dampingCoeff   = mParse.getFloat("dampingCoeff");
    maxVoltage     = mParse.getFloat("maxVoltage");
    inertia        = mParse.getFloat("inertia");
    PID = std::make_unique<PIDController>(    mParse.getFloat("kp"),
                                mParse.getFloat("ki"),
                                mParse.getFloat("kd"),
                                timeStep);


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
    if(coilResistance < 0) throw std::runtime_error("coilResistance Cannot be < 0");
    float phaseCurrent = voltageAcrossCoil / coilResistance;
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

void motor::updateMotorAngularVelocity(float timeStep , float loadTorque, battery& bat, float rad_sec){
    PID->setTarget(rad_sec);
    PID->setGains(.5,.2,0); 
    PID->setTimeStep(timeStep);
    angularVeloRequest = rad_sec;
    float output = PID->update(currentAngularVelocity);
    float volt = output * std::abs(bat.getBatVoltage());

    updateMotor(timeStep,loadTorque,volt);
}




}