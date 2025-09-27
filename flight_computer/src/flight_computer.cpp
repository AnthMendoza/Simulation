#include "../telemetry/include/communication.h"
#include "../telemetry/include/telemetry_packet.h"
#include <iostream>
#include <string>

using namespace std;

int main(){
    flight_computer::telemetry tel(500);
    tel.start();
    string input;
    getline(cin,input);
    while(input != "exit"){

    }
    return 0;
}