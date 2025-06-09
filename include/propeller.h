#pragma once
#include <string>
#include <array>
namespace SimCore{
struct propeller {
    //optional
    std::string name;
    
    // Physical dimensions
    float diameter;       // Total diameter
    float pitchMeters;          // Pitch (distance traveled per rotation)

    float massKg;
    float momentOfInertia;   

    float rpm;
    //prop locations in relation to the nominal center of gravity
    std::array<float,3> location;
    std::array<float,3> locationTransposed;
    //direction is the force vector
    std::array<float,3> direction;
    std::array<float,3> directionTransposed;

    float thrustCoefficient; 
   //true = clockwise : false = counter-clockwise
    bool clockwise;      
};
} //SimCore