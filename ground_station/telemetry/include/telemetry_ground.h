#pragma once

#include <thread>
#include <mutex>
#include <string>
#include "../../../flight_computer/telemetry/include/telemetry_packet.h"
#include "../../../flight_computer/async-sockets/include/udpsocket.hpp"
#include "../../../flight_computer/include/thread_manager.h"


namespace ground_station{
    
#pragma pack(push,1)
struct start_up_packet{
    std::uint32_t magic_number = 0x12345678;
};
#pragma pack(pop)

class telemetry_ground : public thread_manager{

private:
    std::shared_ptr<UDPSocket<>> udp_bridge;
    std::mutex packet_mutex;
    telemetry_packet packet;
    std::string m_interface;
    start_up_packet start_packet;
    int m_baudrate;
    bool m_connected;

public:
    
    telemetry_ground(float packet_rate);

    ~telemetry_ground() = default;

    virtual void thread_proccess() override;

    virtual void thread_startup_proccess() override;

    void start_up();

    void call_back();

    template<typename packetType>
    void send(const packetType& packet) {
        const char* rawData = reinterpret_cast<const char*>(&packet);

        std::string packetStr(rawData, sizeof(packetType));

        udp_bridge->Send(packetStr);
    }

    void receive(std::vector<uint8_t>& buffer);

    inline bool is_connected() const{
        return m_connected;
    }
    
    inline void close()const{
        udp_bridge->Close();
    }

    inline telemetry_packet get_telemetry_packet(){
        return packet;
    }



    bool valid_packet(const char* message, int length);

    bool validate_start_up(ground_station::start_up_packet& start_packet);
};


}