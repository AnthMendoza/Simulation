#pragma once

#include <thread>
#include <mutex>
#include "../../flight_computer/include/telemetry_packet.h"
#include "../../flight_computer/async-sockets/include/udpsocket.hpp"

constexpr auto CONFIG_PATH      = "../config/communication.toml";
constexpr auto TARGET           = "telemetry";
constexpr auto PORT_ACCESS_NAME = "port";
constexpr auto IP_ADDRESS = "IP";
namespace ground_station{
class telemetry_ground {
public:
    
    telemetry_ground();

    ~telemetry_ground() = default;

    void call_back();

    template<typename packetType>
    void send(const packetType& packet) {
        const char* rawData = reinterpret_cast<const char*>(&packet);

        std::string packetStr(rawData, sizeof(packetType));

        udp_bridge->Send(packetStr);
    }

    void receive(std::vector<uint8_t>& buffer);

    bool is_connected() const;
 
    void close();

    inline telemetry_packet get_telemetry_packet(){
        return packet;
    }

private:
    std::shared_ptr<UDPSocket<>> udp_bridge;
    std::mutex packet_mutex;
    telemetry_packet packet;
    std::string m_interface;
    int m_baudrate;
    bool m_connected;
};

}