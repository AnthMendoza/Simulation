#include "../../include/parameterEstimation/motorParameter.h"
#include <string>

int main(){
    std::string batteryConfigPath = "../configs/Battery_Config.toml";
    std::string propellerConfigPath = "../configs/propeller.toml";
    std::string motorConfigPath = "../configs/motor_MN1010.toml";

    parameterEstimation::motorEstimation estimation(motorConfigPath,batteryConfigPath,propellerConfigPath);
    estimation.test();

    return 0;
}   