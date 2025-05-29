#include "../include/battery.h"
#include <algorithm>
namespace SimCore{

    battery::battery(float capacityAh, float initialSoc = 1.0f): capacityAh(capacityAh), soc(initialSoc){
        init();
    }

    void battery::init(){

    }
    // Update soc based on current draw. negative means discharge
    void battery::updateBattery(float currentAmps) {
        if(soc <= safetyTerminationLevel){
            charged = false;
            return;
        }

        //3600 seconds = 60 minutes * 60 seconds in an hour
        float deltaAh = (currentAmps * timestep) / 3600.0f;  // Convert to Ah
        wattHours += voltage * wattHours;
        soc -= deltaAh / capacityAh;
        
        updateVoltage();
    }


    float battery::getRemainingCapacityAh() const{
        return soc * capacityAh;
    }

    void battery::setSOC(float newSoc) {
        soc = std::clamp(newSoc, 0.0f, 1.0f);
    }

    void battery::updateVoltage(){
        voltage = 1;
    }


}