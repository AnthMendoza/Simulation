#include "../../include/subsystems/battery.h"
#include "../../include/sim/toml.h"
#include <algorithm>
#include <vector>
#include <iostream>
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
        toml::tomlParse bParse;
        bParse.parseConfig( config,"battery");
        nominalInternalResistance   = bParse.getFloat("nominalInternalResistance");
        capacityAh                  = bParse.getFloat("capacityAh");
        nominalVoltage              = bParse.getFloat("nominalVoltage");
        cellCount                   = bParse.getFloat("cellCount");
        currentCapacity             = bParse.getFloat("currentCapacity");
        soc                         = bParse.getFloat("soc");
        safetyTerminationLevel      = bParse.getFloat("safteyTerminationLevel");
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