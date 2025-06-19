#pragma once
#include <string>
#include <array>
#include <cmath>
#include "toml.h"
namespace SimCore{
struct propeller {
    //optional
    std::string name;
    
    // Physical dimensions
    float diameter;       // Total diameter
    float pitchMeters;          // Pitch (distance traveled per rotation)

    float massKg;
    float momentOfInertia;   
    //rad/s
    //prop locations in relation to the nominal center of gravity
    std::array<float,3> location;
    std::array<float,3> locationTransposed;
    //direction is the force vector
    std::array<float,3> direction;
    std::array<float,3> directionTransposed;

    float thrustCoefficient;
    float powerCoefficient;
    float dragCoefficient;
   //true = clockwise : false = counter-clockwise
    bool clockwise;
    //Drag Torque and thrust force is as defined in the paper "Generalized Control Allocation Scheme for Multirotor Type of UAVs"
    //Kotarski, Denis & Kasac, Josip. (2018). Generalized Control Allocation
    //Scheme for Multirotor Type of UAVs. 10.5772/intechopen.73006. 
    inline propeller(std::string& propellerConfig){
        initPropeller(propellerConfig);
    }
    inline float dragTorque(float airDensity ,float angularVelocity){
        float k_t =  powerCoefficient * airDensity * M_PI * pow(diameter/2, 5);
        return k_t * pow(angularVelocity,3);  
    }
    inline float thrustForce(float airDensity , float angularVelocity){
        if(angularVelocity <0) return 0;
        float k_f = thrustCoefficient * airDensity * M_PI * pow(diameter/2,2) * pow(diameter/2,2);
        return k_f * pow(angularVelocity,2);
    }
    //Sets prop attributes to config presets
    inline void initPropeller(std::string& propellerConfig){
    toml::tomlParse propParse;
    propParse.parseConfig(propellerConfig,"propeller");
    diameter = propParse.getFloat("diameter");
    massKg = propParse.getFloat("massKg");
    momentOfInertia = propParse.getFloat("MOI");
    pitchMeters = propParse.getFloat("pitch");
    thrustCoefficient = propParse.getFloat("thrustCoefficient");
    dragCoefficient = propParse.getFloat("dragCoefficient");  
    powerCoefficient = dragCoefficient;  
    }
    /// @brief 
    /// @param airDensity 
    /// @param thrustRequest 
    /// @return if thrust request is negative the returned value will be zero due to a sqrt of the value.
    inline float desiredAngularVelocity(float airDensity,float thrustRequest){
        float k = M_PI * airDensity * thrustCoefficient * pow(diameter/2,4);
        if (k <= 0) return 0;
        float v = thrustRequest/k;
        if(v < 0 ) return 0;
        return sqrt(v);
    }
};
//Sets prop attributes to config presets


} //SimCore