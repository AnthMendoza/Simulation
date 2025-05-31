#include "../include/motor.h"
#include "../include/toml.h"
#include <vector>
#include <string>


namespace SimCore{
motor::motor(std::string& config){
    init(config);
}

void motor::init(std::string& config){
    toml::tomlParse mParse;
    mParse.parseConfig(config ,"motor");
    freeSpeedRpm   = mParse.floatValues["freeSpeedRpm"];
    stallTorque    = mParse.floatValues["stallTorque"];          
    stallCurrent   = mParse.floatValues["stallCurrent"];      
    noLoadCurrent  = mParse.floatValues["noLoadCurrent"];   
    resistance     = mParse.floatValues["resistance"];
    voltage        = mParse.floatValues["voltage"];           
    kv             = mParse.floatValues["kv"];                  
    kt             = mParse.floatValues["kt"];
            
}

void motor::update(){
    if(!isEnabled) currentRpm = 0;
}
//sets currentRequest to the motor current ensureing its not over maxCurrent;
void motor::setCurrent(float currentRequest){
    if(currentRequest <= stallCurrent) currentCurrent = currentRequest;
    else currentCurrent = stallCurrent;
}

void motor::rpmRequest(float rpm){

}


}