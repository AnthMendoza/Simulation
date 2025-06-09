#include "../include/drone.h"
#include <string>
using namespace SimCore;
int main(){

    std::string batteyConfig , motorConfig , droneConfig;
    droneControl drone;
    drone.init(batteyConfig,motorConfig ,droneConfig);
    drone.body->setSquare(0.5f,0.5f,0.0005f);
    drone.body->offsetCOG({0,0,0.005f});


    return 1;
}