#include "../include/motor.h"
#include "../include/toml.h"
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>


namespace SimCore{
motor::motor(std::string& config,float timeStep){
    init(config,timeStep);
}

void motor::init(std::string& config, float timeStep){
    toml::tomlParse mParse;
    mParse.parseConfig(config ,"motor");
    freeSpeedAngularVelocity   = mParse.floatValues["freeSpeedAngularVelocity"];
    stallTorque    = mParse.floatValues["stallTorque"];          
    stallCurrent   = mParse.floatValues["stallCurrent"];      
    noLoadCurrent  = mParse.floatValues["noLoadCurrent"];   
    coilResistance = mParse.floatValues["coilResistance"];
    voltage        = mParse.floatValues["voltage"];           
    kv             = mParse.floatValues["kv"];                  
    kt             = mParse.floatValues["kt"];
    dampingCoeff   = mParse.floatValues["dampingCoeff"];
    maxVoltage     = mParse.floatValues["maxVoltage"];
    inertia        = mParse.floatValues["inertia"];
    PID = std::make_unique<PIDController>(  mParse.floatValues["kp"],
                                            mParse.floatValues["ki"],
                                            mParse.floatValues["kd"],
                                            timeStep);
    PID->setOutputLimits(-1,1);

    currentAngularVelocity = 0;
    ke = kt;

            
}
//control loop modulates voltage input
void motor::angualrVeloctiyRequest(float rad_per_sec){
    angualrVeloRequest = rad_per_sec;
}


void motor::updateMotor(float timeStep , float loadTorque, float voltage) {
    setVoltage(voltage);
    // Calculate back-EMF
    backEMF = ke * currentAngularVelocity;

    float voltageAcrossCoil = appliedVoltage - backEMF;
    float phaseCurrent = voltageAcrossCoil / coilResistance;
    currentCurrent = phaseCurrent;

    motorTorque = kt * phaseCurrent;
    float dampingTorque = dampingCoeff * currentAngularVelocity;
    netTorque = motorTorque - loadTorque - dampingTorque;
    
    // integrate
    float angularAcceleration = netTorque / inertia;
    currentAngularVelocity += angularAcceleration * timeStep;
    
    electricalPower = appliedVoltage * phaseCurrent;
    mechanicalPower = motorTorque * currentAngularVelocity;
    efficiency = (electricalPower > 0.01f) ? (mechanicalPower / electricalPower): 0.0f;
}

void motor::updateMotorAngularVelocity(float timeStep , float loadTorque, battery& bat, float rad_sec){
    PID->setTarget(rad_sec);
    float output = PID->update(currentAngularVelocity);
    float volt = output * bat.getBatVoltage();
    updateMotor(timeStep,loadTorque,volt);
}



}