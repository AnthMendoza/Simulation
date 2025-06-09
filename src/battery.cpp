#include "../include/battery.h"
#include "../include/toml.h"
#include <algorithm>
#include "motor.h"
#include <vector>
namespace SimCore{

    battery::battery(){
    }

    void battery::init(std::string& config){
        toml::tomlParse bParse;
        bParse.parseConfig( config,"battery");
        nominalInternalResistance   = bParse.floatValues["nominalInternalResistance"];
        capacityAh                  = bParse.floatValues["capacityAh"];
        nominalVoltage              = bParse.floatValues["nominalVolatge"];
        cellCount                   = bParse.floatValues["cellCount"];
        wattHours                   = bParse.floatValues["wattHours"];
        currentCapacity             = bParse.floatValues["currentCapactiy"];
        soc                         = bParse.floatValues["soc"];
        safetyTerminationLevel      = bParse.floatValues["safteyTerminationLevel"];

    }

    // Update soc based on current draw. negative means discharge
    void battery::updateBattery() {
        if(soc <= safetyTerminationLevel){
            charged = false;
            return;
        }

        //3600 seconds = 60 minutes * 60 seconds in an hour
        float deltaAh = (currentDraw * timestep) / 3600.0f;  // Convert to Ah
        wattHours += voltage * wattHours;
        soc -= deltaAh / capacityAh;
        
        updateVoltage();
    }


    float battery::getRemainingCapacityAh() const{
        return soc * capacityAh;
    }

    void battery::updateVoltage(){
        voltage = 1;
    }

    //Current Request is the highlevel request from control. The battery then limits the current based on battery current spec.
    void battery::currentBalancing(std::vector<std::unique_ptr<motor>> motors){
        float totalCurrentRequest = 0;
        for(int i = 0 ; i < motors.size() ; i++){
            totalCurrentRequest += motors[i]->getCurrentCurrent();
        }
        if(totalCurrentRequest <= currentCapacity){
            return;
        } 
        std::vector<size_t> indices(motors.size());
        for (size_t i = 0; i < indices.size(); ++i)
            indices[i] = i;

        std::sort(indices.begin(), indices.end(),[&motors](size_t a, size_t b) {return motors[a]->getCurrentCurrent() < motors[b]->getCurrentCurrent();});
        for(auto index:indices){
            float avgCurrent = totalCurrentRequest/motors.size();
            if(motors[index]->getCurrentCurrent() > avgCurrent){
                motors[index]->setCurrent(avgCurrent);
            }
            totalCurrentRequest -= motors[index]->getCurrentCurrent();
        }
    }
}