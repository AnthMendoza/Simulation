#include "../include/communication_base.h"
#include <util/toml.h>
#include <string>
#include <iostream>
#include <cstring>

using namespace std;

constexpr auto TARGET           = "telemetry";
constexpr auto PORT_ACCESS_NAME = "port";



avionics::telemetry_base::telemetry_base(telem_ring& tel_buff,std::string config_path): thread_manager(), tel_buffer(tel_buff) , CONFIG_PATH(config_path){

}

void avionics::telemetry_base::startup_helper(){
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

void avionics::telemetry_base::call_back(){
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

void avionics::telemetry_base::send_packet(){
    auto now = std::chrono::steady_clock::now();
    float now_seconds = std::chrono::duration<float>(now - start_time).count();


    telemetry_scheduler(now_seconds);

    
}

bool avionics::telemetry_base::validate_start_up(ground_station::start_up_packet& start_packet){
    ground_station::start_up_packet reference_packet;
    if(reference_packet.magic_number == start_packet.magic_number){
        return true;
    }
    return false;
}


void avionics::telemetry_base::thread_process(){
    telemetry_scheduler(start_to_recent_call_time());    
    
}

void avionics::telemetry_base::thread_startup_process(){
    initialize_base();
    startup_helper();
    set_scheduler(telemetry_scheduler);
}




