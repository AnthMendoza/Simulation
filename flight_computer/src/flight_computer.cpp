#include "../include/communication.h"
#include "../include/telemetry_packet.h"
#include <iostream>

using namespace std;

int main(){
    telemetry_packet packet{};
    packet.payload.position.altitude = 20.0f;
    flight_computer::telemetry tel;
    tel.set_packet(packet);

    while(1){
        tel.send_packet();
    }
}