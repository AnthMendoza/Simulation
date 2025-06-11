#include "../include/drone.h"
#include "../include/propeller.h"
#include <string>
using namespace SimCore;
int main(){

    std::string batteyConfig , motorConfig , droneConfig;
    droneBody drone;
    drone.init(batteyConfig,motorConfig ,droneConfig);
    propeller prop;
    drone.setSquare(0.5f,0.5f,prop);
    drone.offsetCOG({0,0,0.005f});


    return 1;
}