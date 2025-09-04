#include "../include/communication.h"
#include "../../simulation/include/sim/toml.h"
#include "../../simulation/include/utility/utility.h"
#include <string>
#include <iostream>
#include <cstring>

using namespace std;



flight_computer::telemetry::telemetry(){
    std::string contents = readFileAsString(CONFIG_PATH);

    toml::tomlParse coms_toml;
    coms_toml.parseConfig(contents,TARGET);

    udp_bridge = std::make_shared<UDPServer<>>();

    auto port = static_cast<uint16_t>(coms_toml.getFloat(PORT_ACCESS_NAME));

    udp_bridge->Bind(port,[](int errorCode, string errorMessage)
    { cout << errorCode << " : " << errorMessage << endl;});

}

void flight_computer::telemetry::call_back(){
    std::lock_guard<std::mutex> lock(packet_mutex);
    udp_bridge->onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
        
        if (length < sizeof(telemetry_packet)) {
            std::cerr << "Packet too small!" << std::endl;
            return;
        }

        memcpy(&packet, message, sizeof(telemetry_packet));

        std::cout << "Packet id: " << packet.payload.position.altitude << ", value: " << packet.header.crc16 << std::endl;
    };
}

void flight_computer::telemetry::send_packet(){
    std::lock_guard<std::mutex> lock(packet_mutex);
    send(packet);
}

