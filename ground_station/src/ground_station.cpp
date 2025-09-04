#include "../include/telemetry_ground.h"
#include "../../flight_computer/include/telemetry_packet.h"
#include <iostream>
#include <string>
using namespace std;
int main(){
    ground_station::telemetry_ground telemetry;
    telemetry_packet packet;
    std::memset(&packet,0,sizeof(telemetry_packet));
    string input;
    getline(cin, input);
    while (input != "exit"){
        getline(cin,input);
    }


    return 0;
}