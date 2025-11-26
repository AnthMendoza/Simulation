#pragma once

#include <thread>
#include <mutex>
#include <string>
#include "../../../flight_computer/telemetry/include/telemetry_packet.h"
#include "../../../flight_computer/third_party/async-sockets/include/udpsocket.hpp"
#include "../../../flight_computer/include/thread_manager.h"
#include "packet_threaded.h"


namespace ground_station{
    
#pragma pack(push,1)
struct start_up_packet{
    std::uint32_t magic_number = 0x12345678;
};

struct ack_packet{
    std::uint32_t ack = 0xAABBCCDD;
};


#pragma pack(pop)

using packet_mutex = utility::telemetry::packet_with_mutex<telemetry_packet_gui>;

using packet_mutex_ptr = std::shared_ptr<packet_mutex>;


class telemetry_ground : public thread_manager{

private:
    std::shared_ptr<UDPSocket<>> udp_bridge;
    packet_mutex_ptr packet_struct;
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

    inline telemetry_packet_gui get_telemetry_packet(){
        return packet_struct->get_packet();
    }

    inline void init_packet_struct(){
        packet_struct = std::make_shared<packet_mutex>();
        telemetry_packet_gui pack = {};

        pack.attitude.pitch = 1;
        pack.attitude.roll = 2;
        pack.attitude.yaw = 3;
        pack.position.x = 10;
        pack.position.y = 20;
        pack.position.z = 30;
        pack.velocity.vx = 10;
        pack.velocity.vy = 0;
        pack.velocity.vz = 0;

        packet_struct->set_packet(pack);
    }

    inline packet_mutex_ptr transfer_packet_struct(){
        return packet_struct;
    }

    bool valid_packet(const char* message, int length);

    bool validate_start_up(ground_station::start_up_packet& start_packet);

};


}