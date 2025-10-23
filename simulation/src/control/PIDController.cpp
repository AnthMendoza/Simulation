#include "../../include/control/PIDController.h"
#include <algorithm>
#include <iostream>
PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd),
      integral(0.0f), previousError(0.0f),
      minOutput(-1e10f), maxOutput(1e10f),target(0), clampInt(2.0f),previousSample(0) {
    if (this->kp == 0.0f && this->ki == 0.0f && this->kd == 0.0f) {
        std::cout << "Warning: all gains in PID loop within motor are 0. Setting to default values. kp=0.5 ki=0.2 kd=0.0\n";
        this->kp = 0.5f;
        this->ki = 0.2f;
        this->kd = 0.0f;
    }
}

PIDController::PIDController(const PIDController& other): kp(other.kp), ki(other.ki), kd(other.kd),
      target(other.target), integral(other.integral), previousError(other.previousError),
      minOutput(other.minOutput), maxOutput(other.maxOutput), previousSample(other.previousSample),clampInt(other.clampInt) {}




void PIDController::setGains(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    minOutput = min;
    maxOutput = max;
}

void PIDController::reset() {
    integral = 0.0f;
    previousError = 0.0f;
    previousSample = 0.0f;
}


void PIDController::setTarget(float t){
    target = t;
}

float PIDController::update(const float& measurement , const float deltaTime) {
    float error = target - measurement;
    integral += error * deltaTime;
    integral = std::clamp(integral,-clampInt,clampInt);
    float derivative = 0;
    
    if(deltaTime <= 0){
        std::cout<<"Warning bypassing derivvative in PID. delta time <= 0 "<< deltaTime<<"\n";
    }
    else{
        derivative = -(measurement - previousSample) / deltaTime;
    }
    
    float output = kp * error + ki * integral + kd * derivative;
    output = std::clamp(output, minOutput, maxOutput);
    previousSample = measurement;
    previousError = error;
    return output;
}
