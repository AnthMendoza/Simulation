#include "../include/battery.h"
#include "../include/toml.h"
#include <algorithm>
#include <vector>
namespace SimCore{

    battery::battery(std::string& config){
        init(config);
    }

    void battery::init(std::string& config){
        toml::tomlParse bParse;
        bParse.parseConfig( config,"battery");
        nominalInternalResistance   = bParse.floatValues["nominalInternalResistance"];
        capacityAh                  = bParse.floatValues["capacityAh"];
        nominalVoltage              = bParse.floatValues["nominalVoltage"];
        cellCount                   = bParse.floatValues["cellCount"];
        wattHours                   = bParse.floatValues["wattHours"];
        currentCapacity             = bParse.floatValues["currentCapactiy"];
        soc                         = bParse.floatValues["soc"];
        safetyTerminationLevel      = bParse.floatValues["safteyTerminationLevel"];
        voltage = nominalVoltage;
        socVoltage = voltage;

    }

    // Update soc based on current draw. negative means discharge
    void battery::updateBattery(float current) {
        if(soc <= safetyTerminationLevel){
            charged = false;
            return;
        }

        //3600 seconds = 60 minutes * 60 seconds in an hour
        float deltaAh = (currentDraw * timestep) / 3600.0f;  // Convert to Ah
        wattHours += voltage * wattHours;
        soc -= deltaAh / capacityAh;

        updateVoltage(current);
    }


    float battery::getRemainingCapacityAh() const{
        return soc * capacityAh;
    }

    void battery::updateVoltage(float current){
        current = abs(current);
        currentDraw = current;
        if(currentDraw * nominalInternalResistance > socVoltage){
            voltage = 0;
            return;
        }
        voltage = socVoltage - (currentDraw * nominalInternalResistance);
    }
}