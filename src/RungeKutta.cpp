#include <cmath>
#include "../include/RungeKutta.h"

float acceleration(float force , float mass){
    return force / mass;
}

void RungeKutta4th(float force, float mass, float timeStep, float &velocity, float &position) {
    

    // Calculate k1 for velocity and position
    float k1v = acceleration(force , mass) * timeStep;
    float k1x = velocity * timeStep;

    // Calculate k2 for velocity and position
    float k2v = (acceleration(force , mass) + k1v / 2.0f) * timeStep;
    float k2x = (velocity + k1x / 2.0f) * timeStep;

    // Calculate k3 for velocity and position
    float k3v = acceleration(force , mass) + k2v / 2.0f * timeStep;
    float k3x = (velocity + k2x / 2.0f) * timeStep;

    // Calculate k4 for velocity and position
    float k4v = (acceleration(force , mass)+ k3v) * timeStep;
    float k4x = (velocity + k3x) * timeStep;

    // weighted averge of the samples
    velocity = velocity + (k1v + 2 * k2v + 2 * k3v + k4v) / 6.0f;
    position = position + (k1x + 2 * k2x + 2 * k3x + k4x) / 6.0f;
}