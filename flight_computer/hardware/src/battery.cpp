#include <battery.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <util/yaml.h>
#include <battery_modeling.h>

namespace hardware{

    battery::battery(std::string& config){
        init(config);
    }

    void battery::init(std::string& config){
        YAML::Node configNode = YAML::LoadFile(config);

        const auto& battery = configNode["battery"];

        nominalInternalResistance = utility::getRequired<float>(battery,"nominalInternalResistance","battery");
        totalCapacityAh             = utility::getRequired<float>(battery,"totalCapacityAh","battery");
        cellCount                 = utility::getRequired<float>(battery,"cellCount","battery");
        currentCapacityAh           = utility::getRequired<float>(battery,"currentCapacityAh","battery");
        safetyTerminationLevel    = utility::getRequired<float>(battery,"safetyTerminationLevel","battery");
        nominalVoltage = cellCount * static_cast<float>(soc_to_mv((uint16_t)100))/1000;
    }

    static uint16_t calculateSOC(float voltage , uint8_t cellCount){
        auto adjustedVoltageToCell = voltage * 1000 / static_cast<float>(cellCount);
        return mv_to_soc(static_cast<uint16_t>(adjustedVoltageToCell));
    }

    // Update soc based on current draw. negative means discharge
    void battery::updateBattery(float current, float currentTimeSeconds) {
        if(soc <= safetyTerminationLevel){
            charged = false;
        }
        if(soc < 0) soc = 0;
    
        float deltaAh = 0;
        if(!firstSample){
            float dt = std::max(0.0f, currentTimeSeconds - lastTimeSeconds);
            
            deltaAh = (currentDraw * dt) / 3600.0f;
        }
        currentCapacityAh -= deltaAh;

        if(firstSample){
            //big assumption, no major current draw on startup
            soc = calculateSOC(voltage , cellCount);
            //linear maping from sco to remaining charge 
            currentCapacityAh = (static_cast<float>(soc)/100) * totalCapacityAh;
            
        }

        firstSample = false;
        lastTimeSeconds = currentTimeSeconds;

        if(currentCapacityAh <= 0 ){
            currentCapacityAh = .01;
        }

        updateVoltage(current);
    }


    float battery::getRemainingCapacityAh() const{
        return currentCapacityAh;
    }
    //reduce voltage based on current demand. All batteries have voltage sag associated with the internal Resistance
    void battery::updateVoltage(float current){
        currentDraw = abs(current);
        if(currentDraw * nominalInternalResistance > socVoltage){
            voltage = 0;
            return;
        }
        //socVoltage is nominal no current voltage. state of charge defines its nominal current;
        socVoltage = static_cast<float>(soc_to_mv(soc)/1000) * cellCount; 
        voltage = socVoltage - calculateInternalVoltageDrop(currentDraw);
    }

    
    // more complex modeling would include a thermal model for internalResistance.
    // Leaving this as a standalone method for future implementations.
    float battery::calculateInternalVoltageDrop(float current) const{
        return (current * nominalInternalResistance);
    }

    bool battery::manuallySetInitVoltage(float voltageV){
        voltage = voltageV;
        soc = calculateSOC(voltage,cellCount);
        
    }

    void battery::setBatterySpecs(float _totalCapacityAh, float _voltage, int _cellCount){
        totalCapacityAh = _totalCapacityAh;
        if(_voltage >= soc_to_mv((uint16_t)0) && _voltage <= soc_to_mv((uint16_t)100)){
            manuallySetInitVoltage(nominalVoltage);
        }
        cellCount = _cellCount;
    }

    void battery::setInternalResistance(float resistance){
        internalResistance = std::max(0.0f,resistance);
    }

    void battery::setSafetyLimits(float terminationLevel, float _maxDischargeCurrent,float maxChargeCurrent, float maxTemp, float minTemp){
        safetyTerminationLevel = terminationLevel;
        maxDischargeCurrent = _maxDischargeCurrent;
    }

    
}