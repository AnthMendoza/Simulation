#include "../include/PIDController.h"
#include <algorithm>

PIDController::PIDController(float kp, float ki, float kd, float dt)
    : kp(kp), ki(ki), kd(kd), dt(dt),
      integral(0.0f), previousError(0.0f),
      minOutput(-1e10f), maxOutput(1e10f) {}

void PIDController::setGains(float kp, float ki, float kd) {
    kp = kp;
    ki = ki;
    kd = kd;
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
    float error = target - measurement;
    integral += error * dt;
    float derivative = (error - previousError) / dt;

    float output = kp * error + ki * integral + kd * derivative;
    output = std::clamp(output, minOutput, maxOutput);

    previousError = error;
    return output;
}
