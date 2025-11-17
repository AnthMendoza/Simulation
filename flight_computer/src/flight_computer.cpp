#include "../telemetry/include/communication.h"
#include "../telemetry/include/telemetry_packet.h"
#include <iostream>
#include <string>

using namespace std;

#define RATE_MS 500

int main(){

    flight_computer::telemetry tel(RATE_MS);
    tel.start();
    string input;
    getline(cin,input);

    while(input != "exit"){
    }
    
    return 0;
}