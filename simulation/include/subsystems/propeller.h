#pragma once
#include <string>
#include <array>
#include <cmath>
#include <util/yaml.h>
#include <base/base.h>


namespace SimCore{
class propeller{
    public:
    //prop locations in relation to the nominal center of gravity
    units::vec3 location;

    std::string name;
    
    // Physical dimensions
    float diameter;       // Total diameter
    float pitchMeters;          // Pitch (distance traveled per rotation)
    int rotationDirection = 1;
    float massKg;
    float momentOfInertia;   
    //rad/s
    //transpose Locations are rotated with the vehicle. 
    units::vec3 locationTransposed;
    //size taken from location and used to renormilze locationTransposed
    float locationVectorLength;
    //direction is the force vector
    units::vec3 direction;
    units::vec3 directionTransposed;
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

    inline float dragTorque(float airDensity, float angularVelocity) const{
        if(diameter <= 0) throw std::runtime_error("Diameter cannot be <= 0");
        float radius = diameter * 0.5f;
        float k_t = powerCoefficient * airDensity * M_PI * pow(radius, 5);
        float drag = k_t * angularVelocity * angularVelocity;
        return drag;
    }

    inline float thrustForce(float airDensity, float angularVelocity) const{
        if (angularVelocity < 0) return 0;
        if(diameter <= 0) throw std::runtime_error("Diameter cannot be <= 0");
        float radius = diameter * 0.5f;
        float k_f = thrustCoefficient * airDensity * M_PI * pow(radius, 4);
        return k_f * angularVelocity * angularVelocity;
    }
    //Sets prop attributes to config presets
    void initPropeller(std::string& propellerConfig){
        YAML::Node node = YAML::LoadFile(propellerConfig);
        const auto& propeller = node["propeller"];

        diameter          = utility::getRequired<float>(propeller, "diameter", "propeller");
        massKg            = utility::getRequired<float>(propeller, "massKg", "propeller");
        momentOfInertia   = utility::getRequired<float>(propeller, "MOI", "propeller");
        pitchMeters       = utility::getRequired<float>(propeller, "pitch", "propeller");
        thrustCoefficient = utility::getRequired<float>(propeller, "thrustCoefficient", "propeller");
        powerCoefficient  = utility::getRequired<float>(propeller, "powerCoefficient", "propeller");
    }

    /// @brief 
    /// @param airDensity 
    /// @param thrustRequest 
    /// @return if thrust request is negative the returned value will be zero due to a sqrt of the value.

    inline float desiredAngularVelocity(float airDensity,float thrustRequest){
        if(diameter <= 0) throw std::runtime_error("Diameter cannot be <= 0");
        float k = M_PI * airDensity * thrustCoefficient * pow(diameter/2,4);
        if (k <= 0) return 0;
        float v = thrustRequest/k;
        if(v < 0 ) return 0;
        return sqrt(v);
    }

    inline units::vec3 getLocation() const{
        return location;
    }

    inline void setLocation(units::vec3 loc){
        location = loc;
        locationVectorLength = vectorMag(location);
    }
};
//Sets prop attributes to config presets


} //SimCore