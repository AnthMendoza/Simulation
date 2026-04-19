#include "../../include/subsystems/battery.h"
#include <util/toml.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <util/yaml.h>

namespace SimCore{

    battery::battery(std::string& config){
        init(config);
    }

    battery::battery(const battery& other) 
    :   configBattery(other.configBattery),
        capacityAh(other.capacityAh),
        nominalVoltage(other.nominalVoltage),
        cellCount(other.cellCount),
        maxVoltagePerCell(other.maxVoltagePerCell),
        minVoltagePerCell(other.minVoltagePerCell),
        internalResistance(other.internalResistance),
        nominalInternalResistance(other.nominalInternalResistance),
        wattHours(other.wattHours),
        currentCapacity(other.currentCapacity),
        soc(other.soc),
        voltage(other.voltage),
        currentDraw(other.currentDraw),
        cycleCount(other.cycleCount),
        socVoltage(other.socVoltage),
        safetyTerminationLevel(other.safetyTerminationLevel),
        maxDischargeCurrent(other.maxDischargeCurrent),
        charged(other.charged),
        totalEnergyDelivered(other.totalEnergyDelivered),
        totalEnergyCharged(other.totalEnergyCharged),
        peakDischargeCurrent(other.peakDischargeCurrent),
        lastTimeSeconds(0),
        firstSample(true)
    {
        
    }

    void battery::init(std::string& config){
        YAML::Node configNode = YAML::LoadFile(config);

        const auto& battery = configNode["battery"];

        nominalInternalResistance = utility::getRequired<float>(battery,"nominalInternalResistance","battery");
        capacityAh                = utility::getRequired<float>(battery,"capacityAh","battery");
        nominalVoltage            = utility::getRequired<float>(battery,"nominalVoltage","battery");
        cellCount                 = utility::getRequired<float>(battery,"cellCount","battery");
        currentCapacity           = utility::getRequired<float>(battery,"currentCapacity","battery");
        soc                       = utility::getRequired<float>(battery,"soc","battery");
        safetyTerminationLevel    = utility::getRequired<float>(battery,"safetyTerminationLevel","battery");

        voltage = nominalVoltage;
        socVoltage = voltage;
        wattHours = capacityAh * nominalVoltage;

    }

    // Update soc based on current draw. negative means discharge
    void battery::updateBattery(float current, float currentTimeSeconds) {
        if(soc <= safetyTerminationLevel){
            charged = false;
        }
        if(soc < 0) soc = 0;
    
        float deltaAh = 0;
        if(!firstSample){
            float dt = currentTimeSeconds - lastTimeSeconds;
            deltaAh = (currentDraw * dt) / 3600.0f;
        }
        firstSample = false;
        lastTimeSeconds = currentTimeSeconds;

        if(capacityAh <= 0 ){
            capacityAh = .01;
            std::cout<<"Battery is dead";
        }
        soc -= deltaAh / capacityAh;

        updateVoltage(current);
    }


    float battery::getRemainingCapacityAh() const{
        return soc * capacityAh;
    }
    //reduce voltage based on current demand. All batteries have voltage sag associated with the inetal Resistance
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