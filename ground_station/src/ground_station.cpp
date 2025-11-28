#include "../telemetry/include/telemetry_ground.h"
#include "../../flight_computer/telemetry/include/telemetry_packet.h"
#include "../gui/include/gui_connector.h"
#include "../include/ground_station_startup.h"
#include <iostream>
#include <string>

using namespace std;

int main(){
    ground_station::ground_station_startup ground_station_instance;
    
    ground_station_instance.start();
    ground_station::telemetry_ground telemetry(500); 
    gui_connector::gui_connector gui(500);
    gui.set_packet_ptr(telemetry.transfer_packet_struct());
    gui.start();
    telemetry.start();
    string input;
    getline(cin,input);
    while(input != "exit"){

    }
    return 0;
}