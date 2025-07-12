#pragma once
#include <string>
#include <array>
#include <cmath>
#include "../sim/toml.h"
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
    //transpose Locations are rotated with the vehicle. 
    std::array<float,3> locationTransposed;
    //direction is the force vector
    std::array<float,3> direction;
    std::array<float,3> directionTransposed;
    //min = first  ::  max = second .Thrust dyno program. It is motor and battery dependent.
    std::pair<float,float> thrustLimits;
    float thrustCoefficient;
    float powerCoefficient;
   //true = clockwise : false = counter-clockwise
    bool clockwise;
    //Drag Torque and thrust force is as defined in the paper "Generalized Control Allocation Scheme for Multirotor Type of UAVs"
    //Kotarski, Denis & Kasac, Josip. (2018). Generalized Control Allocation
    //Scheme for Multirotor Type of UAVs. 10.5772/intechopen.73006. 
    inline propeller(std::string& propellerConfig){
        initPropeller(propellerConfig);
    }

    inline float dragTorque(float airDensity, float angularVelocity) {
        float radius = diameter * 0.5f;
        float k_t = powerCoefficient * airDensity * M_PI * pow(radius, 5);
        return k_t * angularVelocity * angularVelocity;
    }

    inline float thrustForce(float airDensity, float angularVelocity) const{
        if (angularVelocity < 0) return 0;
        float radius = diameter * 0.5f;
        float k_f = thrustCoefficient * airDensity * M_PI * pow(radius, 4);
        return k_f * angularVelocity * angularVelocity;
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
    powerCoefficient = propParse.getFloat("powerCoefficient");   
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