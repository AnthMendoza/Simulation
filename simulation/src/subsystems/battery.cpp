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
        timestep(other.timestep),
        totalEnergyDelivered(other.totalEnergyDelivered),
        totalEnergyCharged(other.totalEnergyCharged),
        peakDischargeCurrent(other.peakDischargeCurrent)
    {

    }

    void battery::init(std::string& config){
        toml::tomlParse bParse;
        bParse.parseConfig( config,"battery");
        nominalInternalResistance   = bParse.getFloat("nominalInternalResistance");
        capacityAh                  = bParse.getFloat("capacityAh");
        nominalVoltage              = bParse.getFloat("nominalVoltage");
        cellCount                   = bParse.getFloat("cellCount");
        wattHours                   = bParse.getFloat("wattHours");
        currentCapacity             = bParse.getFloat("currentCapacity");
        soc                         = bParse.getFloat("soc");
        safetyTerminationLevel      = bParse.getFloat("safteyTerminationLevel");
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