#include "../include/drone.h"
using namespace SimCore;
int main(){

    droneControl drone;
    drone.init();
    drone.body->setSquare(0.5f,0.5f,0.0005f);
    drone.body->offsetCOG({0,0,0.005f});


    return 1;
}