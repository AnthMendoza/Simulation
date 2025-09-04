#include "../include/telemetry_ground.h"
#include "../../simulation/include/sim/toml.h"
#include <iostream>

using namespace std;

ground_station::telemetry_ground::telemetry_ground(){
    toml::tomlParse telemetry_toml;
    telemetry_toml.parseConfig(CONFIG_PATH,TARGET);
    

    udp_bridge = make_shared<UDPSocket<>>();
    string IP = telemetry_toml.getString(IP_ADDRESS);
    uint16_t port = static_cast<uint16_t>(telemetry_toml.getFloat(PORT_ACCESS_NAME));
    udp_bridge->Connect(IP,port);
}

void ground_station::telemetry_ground::call_back(){
    std::lock_guard<std::mutex> lock(packet_mutex);
    udp_bridge->onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
        
        if (length < sizeof(telemetry_packet)) {
            std::cerr << "Packet too small!" << std::endl;
            return;
        }

        std::memcpy(&packet, message, sizeof(telemetry_packet));

        std::cout << "Packet id: " << packet.payload.position.altitude << ", value: " << packet.header.crc16 << std::endl;
    };
}


