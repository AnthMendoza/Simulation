#include <cmath>
#include "../include/RungeKutta.h"


void RungeKutta4th(float force, float mass, float timeStep, float &velocity, float &position) {

    auto acceleration = [force, mass](float /* v */) -> float {
        return force / mass;
    };

    // Calculate k1 for velocity and position
    float k1v = acceleration( velocity) * timeStep;
    float k1x = velocity * timeStep;

    // Calculate k2 for velocity and position
    float k2v = acceleration( velocity + k1v / 2.0f) * timeStep;
    float k2x = (velocity + k1v / 2.0f) * timeStep;

    // Calculate k3 for velocity and position
    float k3v = acceleration( velocity + k2v / 2.0f) * timeStep;
    float k3x = (velocity + k2v / 2.0f) * timeStep;

    // Calculate k4 for velocity and position
    float k4v = acceleration( velocity + k3v) * timeStep;
    float k4x = (velocity + k3v) * timeStep;

    // Update velocity and position using weighted sum of k1, k2, k3, and k4
    velocity = velocity + (k1v + 2 * k2v + 2 * k3v + k4v) / 6.0f;
    position = position + (k1x + 2 * k2x + 2 * k3x + k4x) / 6.0f;
}
