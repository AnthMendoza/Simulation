#ifndef COMMUNICATION_H
#define COMMUNICATION_H 


#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include "telemetry_packet.h"
#include "../async-sockets/include/udpserver.hpp"

constexpr auto CONFIG_PATH      = "../config/communication.toml";
constexpr auto TARGET           = "telemetry";
constexpr auto PORT_ACCESS_NAME = "port";

namespace flight_computer{


class telemetry {
public:
    
    telemetry();

    ~telemetry() = default;

    void call_back();

    template<typename packetType>
    void send(const packetType& packet) {
        const char* rawData = reinterpret_cast<const char*>(&packet);

        std::string packetStr(rawData, sizeof(packetType));

        udp_bridge->Send(packetStr);
    }

    void send_packet();

    bool receive(std::vector<uint8_t>& buffer);

    bool is_connected() const;
 
    void close();

    inline void set_packet(telemetry_packet& ref_packet){
        std::lock_guard<std::mutex> lock(packet_mutex);
        packet = ref_packet;
    }
    

private:
    std::shared_ptr<UDPServer<>> udp_bridge;
    std::mutex packet_mutex;
    telemetry_packet packet;
    std::string m_interface;
    int m_baudrate;
    bool m_connected;
};

}

#endif