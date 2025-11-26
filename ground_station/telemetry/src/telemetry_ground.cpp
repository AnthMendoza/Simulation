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
    
    init_packet_struct();

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

        std::optional<telemetry_packet_manager::PayloadVariant> parsed = telemetry_manager.recieved(reinterpret_cast<const uint8_t*>(message),length);

        if(parsed == std::nullopt){
            return;
        }

        m_connected = true;

        handle_payload_variant(*parsed);

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
        start_up();
        std::cout<< "init packet sent from ground. \n";

        return;
    }
}



void ground_station::telemetry_ground::handle_payload_variant(telemetry_packet_manager::PayloadVariant& payload_variant){

    if (auto p = std::get_if<attitude_data>(&payload_variant)) {
        attitude_data data = *p;
        gui_packet->modify([data](telemetry_packet_gui& packet) {
            packet.attitude = data;
        });
        std::cout << "Roll: " << p->roll << ", Pitch: " << p->pitch << ", Yaw: " << p->yaw << std::endl;
    }else if (auto p = std::get_if<position_data>(&payload_variant)) {
        position_data data = *p;
        gui_packet->modify([data](telemetry_packet_gui& packet) {
            packet.position = data;
        });
        std::cout << "X: " << p->x << ", Y: " << p->y << ", Z: " << p->z << std::endl;
    }else if (auto p = std::get_if<velocity_data>(&payload_variant)) {
        velocity_data data = *p;
        gui_packet->modify([data](telemetry_packet_gui& packet) {
            packet.velocity = data;
        });
        std::cout << "VX: " << p->vx << ", VY: " << p->vy << ", VZ: " << p->vz << std::endl;
    }else if (auto p = std::get_if<status_data>(&payload_variant)) {
        status_data data = *p;
        gui_packet->modify([data](telemetry_packet_gui& packet) {
            packet.status = data;
        });
        std::cout << "Status: " << p->state << std::endl;
    }else if (auto p = std::get_if<error_data>(&payload_variant)) {
        error_data data = *p;
        gui_packet->modify([data](telemetry_packet_gui& packet) {
            packet.error = data;
        });
        std::cout << "Error: " << p->code << std::endl;
    }else {
        std::cerr << "Unknown payload type!" << std::endl;
    }

}




