#include "../include/communication.h"
#include "../../../simulation/include/sim/toml.h"
#include "../../../simulation/include/utility/utility.h"
#include "../include/com_utility.h"
#include <string>
#include <iostream>
#include <cstring>

using namespace std;

constexpr auto CONFIG_PATH      = "../config/communication.toml";
constexpr auto TARGET           = "telemetry";
constexpr auto PORT_ACCESS_NAME = "port";



flight_computer::telemetry::telemetry(float packet_rate): thread_manager(packet_rate){
    m_connected = false;
    relay_connected_to_ground = false;
    std::string contents = readFileAsString(CONFIG_PATH);

    toml::tomlParse coms_toml;
    coms_toml.parseConfig(contents,TARGET);

    udp_bridge = std::make_shared<UDPServer<>>();

    auto port = static_cast<uint16_t>(coms_toml.getFloat(PORT_ACCESS_NAME));
    port_tel = port;
    call_back();
    udp_bridge->Bind(port,[](int errorCode, string errorMessage)
    { cout << errorCode << " : " << errorMessage << endl;});

}

void flight_computer::telemetry::call_back(){
    udp_bridge->onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
        if (!m_connected && length == sizeof(ground_station::start_up_packet)) {
            ground_station::start_up_packet reference_packet{0x00000000};
            memcpy(&reference_packet, message, sizeof(reference_packet));
        
            if (validate_start_up(reference_packet)) {
                udp_bridge->Connect(ipv4, port);
                ipv4_tel = ipv4;
                m_connected = true;
                std::cout << "connected to : " << ipv4 << "\n";
            }
            return;
        }
        if(!m_connected) return;
        
        if (length < sizeof(telemetry_packet)) {
            return;
        }

        
        //for return message
    };
}

void flight_computer::telemetry::send_packet(){
    auto packet = packet_mutex.get_packet();
    send(packet);
}

bool flight_computer::telemetry::validate_start_up(ground_station::start_up_packet& start_packet){
    ground_station::start_up_packet reference_packet;
    if(reference_packet.magic_number == start_packet.magic_number){
        return true;
    }
    return false;
}


void flight_computer::telemetry::thread_proccess(){

    if(!is_connected()){
        return;
    }
    
    send_packet();

}

void flight_computer::telemetry::thread_startup_proccess(){
    telemetry_packet packet{};
    auto& attitude = packet.payload.attitude;
    attitude.pitch = 1;
    attitude.roll = 2;
    attitude.yaw = 3;

    auto& pos = packet.payload.position;

    pos.x = 1;
    pos.y = 2;
    pos.z = 3;
    
    
    set_packet(packet);
}

