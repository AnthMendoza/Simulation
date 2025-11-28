#include "../include/communication.h"
#include "../../../simulation/include/sim/toml.h"
#include "../../../simulation/include/utility/utility.h"
#include <string>
#include <iostream>
#include <cstring>

using namespace std;

constexpr auto CONFIG_PATH      = "../../flight_computer/config/communication.toml";
constexpr auto TARGET           = "telemetry";
constexpr auto PORT_ACCESS_NAME = "port";



flight_computer::telemetry::telemetry(): thread_manager(){
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
    auto now = std::chrono::steady_clock::now();
    float now_seconds = std::chrono::duration<float>(now - start_time).count();


    telemetry_scheduler(now_seconds);

    
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
    float interval = 0.050f;
    telemetry_scheduler.add_manager(interval, [this]{ send_ATTITUDE_packet(); });
    interval = 0.100f;
    telemetry_scheduler.add_manager(interval, [this]{ send_POSITION_packet(); });
    interval = 0.050f;
    telemetry_scheduler.add_manager(interval, [this]{ send_VELOCITY_packet(); });
    interval = 3.00f;
    telemetry_scheduler.add_manager(interval, [this]{ send_STATUS_packet(); });
}

constexpr float MAX_SLEEP_TIME = 2.0;

void flight_computer::telemetry::thread_loop() {
    start_time = std::chrono::steady_clock::now();

    thread_startup_proccess();

    auto next_time = std::chrono::steady_clock::now();

    while (running.load()) {  
        thread_proccess();

        auto now = std::chrono::steady_clock::now();
        float now_seconds = std::chrono::duration<float>(now - start_time).count();

        float scheduler_interval = telemetry_scheduler.time_until_next_packet(now_seconds);

        next_time = now;

        if (scheduler_interval > 0) {
            float sleep_time = std::min(scheduler_interval, MAX_SLEEP_TIME);
            next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<float>(sleep_time));
        }
        
        std::this_thread::sleep_until(next_time);
    }
}
