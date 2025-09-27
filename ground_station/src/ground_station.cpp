#include "../telemetry/include/telemetry_ground.h"
#include "../../flight_computer/telemetry/include/telemetry_packet.h"
#include <iostream>
#include <string>
using namespace std;
int main(){
    ground_station::telemetry_ground telemetry(500);
    telemetry.start();
    string input;
    getline(cin,input);
    while(input != "exit"){

    }
    return 0;
}