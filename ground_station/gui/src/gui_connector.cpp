#include "../include/gui_connector.h"
#include <util/util.h>
namespace gui_connector{

constexpr auto CONFIG_PATH      = "../config/communication.toml";
constexpr auto TARGET           = "gui";
constexpr auto PORT_ACCESS_NAME = "port";
constexpr auto IP_ADDRESS = "IP";

gui_connector::gui_connector(float packet_rate) : thread_manager(packet_rate){
    toml::tomlParse gui_toml;
    std::string contents = utility::readFileAsString(CONFIG_PATH);
    gui_toml.parseConfig(contents,TARGET);
    auto port = static_cast<uint16_t>(gui_toml.getFloat(PORT_ACCESS_NAME));
    std::string IP = gui_toml.getString(IP_ADDRESS);
    port_tel = port;
    udp_bridge = std::make_shared<UDPSocket<>>();
    call_back();
    udp_bridge->Connect(IP,port);
}

void gui_connector::thread_process(){
    send_packet();
}

void gui_connector::thread_startup_process(){

}

void gui_connector::send_packet(){
    auto packet = packet_ptr->get_packet();
    send(packet);
}

void gui_connector::call_back(){
    udp_bridge->onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
        
    };
}

}


