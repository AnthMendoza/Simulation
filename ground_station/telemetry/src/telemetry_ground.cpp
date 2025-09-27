#include "../include/telemetry_ground.h"
#include "../../../simulation/include/sim/toml.h"
#include "../../../simulation/include/utility/utility.h"
#include <iostream>

using namespace std;

constexpr auto CONFIG_PATH      = "../config/communication.toml";
constexpr auto TARGET           = "telemetry";
constexpr auto PORT_ACCESS_NAME = "port";
constexpr auto IP_ADDRESS = "IP";

ground_station::telemetry_ground::telemetry_ground(float packet_rate) : thread_manager(packet_rate){
    m_connected = false;

    std::string contents = readFileAsString(CONFIG_PATH);
    
    toml::tomlParse telemetry_toml;
    telemetry_toml.parseConfig(contents,TARGET);
    
    udp_bridge = make_shared<UDPSocket<>>(true);
    string IP = telemetry_toml.getString(IP_ADDRESS);
    uint16_t port = static_cast<uint16_t>(telemetry_toml.getFloat(PORT_ACCESS_NAME));
    call_back();
    udp_bridge->Connect(IP,port);
}

void ground_station::telemetry_ground::call_back(){
    udp_bridge->onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
        if(!m_connected &&  length == sizeof(ground_station::start_up_packet)){
            ground_station::start_up_packet reference_packet{0x00000000};
            memcpy(&reference_packet,message,length);
            if(validate_start_up(reference_packet)){
                m_connected = true;
            }
        }
        std::lock_guard<std::mutex> lock(packet_mutex);
        if(!valid_packet(message,length)) return;
        memcpy(&packet, message, sizeof(telemetry_packet));

        std::cout << "Packet id: " << packet.payload.position.altitude << ", value: " << packet.header.crc16 << std::endl;
    };
}


void ground_station::telemetry_ground::start_up(){

    send(start_packet);
    
}

bool ground_station::telemetry_ground::valid_packet(const char* message, int length){
    if (length < sizeof(telemetry_packet)){
        return false;
    }
    m_connected = true;
    return true;
}

bool ground_station::telemetry_ground::validate_start_up(ground_station::start_up_packet& start_packet){
    ground_station::start_up_packet reference_packet;
    if(reference_packet.magic_number == start_packet.magic_number){
        return true;
    }
    return false;
}

void ground_station::telemetry_ground::thread_startup_proccess(){

}

void ground_station::telemetry_ground::thread_proccess(){
    if(!is_connected()){
        start_up_packet start_packet;
        send(start_packet);
        std::cout<< "init packet from ground\n";
        return;
    }
}





