#include "../include/PIDController.h"
#include <algorithm>
#include <iostream>
PIDController::PIDController(float kp, float ki, float kd, float dt)
    : kp(kp), ki(ki), kd(kd), dt(dt),
      integral(0.0f), previousError(0.0f),
      minOutput(-1e10f), maxOutput(1e10f) {
    
    if (this->kp == 0.0f && this->ki == 0.0f && this->kd == 0.0f) {
        std::cout << "Warning: all gains in PID loop within motor are 0. Setting to default values. kp=0.5 ki=0.2 kd=0.0\n";
        this->kp = 0.5f;
        this->ki = 0.2f;
        this->kd = 0.0f;
    }
}

PIDController::PIDController(const PIDController& other): kp(other.kp), ki(other.ki), kd(other.kd), dt(other.dt),
      target(other.target), integral(other.integral), previousError(other.previousError),
      minOutput(other.minOutput), maxOutput(other.maxOutput) {}




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
}
//For Simultaion with Dynamic timeStep. Planned in the future.
void PIDController::setTimeStep(float timeStep){
    dt = timeStep;
}

void PIDController::setTarget(float t){
    target = t;
}

float PIDController::update(float measurement) {
    previousSample = measurement;
    float error = target - measurement;
    integral += error * dt;
    float derivative = 0;
    if(dt <= 0){
        std::cout<<"Warning bypassing derivvative in PID within a motor. dt <= 0 \n";
    }
    else{
        derivative = (error - previousError) / dt;
    }
    float output = kp * error + ki * integral + kd * derivative;
    output = std::clamp(output, minOutput, maxOutput);

    previousError = error;
    return output;
}
