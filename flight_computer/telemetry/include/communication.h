#ifndef COMMUNICATION_H
#define COMMUNICATION_H 


#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <iostream>
#include <atomic>
#include "telemetry_packet.h"
#include "../../async-sockets/include/udpserver.hpp"
#include "../../../ground_station/telemetry/include/telemetry_ground.h"
#include "../../include/thread_manager.h"

namespace flight_computer{


class telemetry: public thread_manager{

private:
std::shared_ptr<UDPServer<>> udp_bridge;
std::mutex packet_mutex;
telemetry_packet packet;
std::string m_interface;
std::string ipv4_tel;
uint16_t port_tel;
int m_baudrate;
protected:
bool m_connected;
bool relay_connected_to_ground;
public:
    
    telemetry(float packet_rate);


    ~telemetry() = default;

    virtual void thread_proccess() override;

    virtual void thread_startup_proccess() override;

    void call_back();

    template<typename packetType>
    void send(const packetType& packet) {
        const char* rawData = reinterpret_cast<const char*>(&packet);

        std::string packetStr(rawData, sizeof(packetType));

        udp_bridge->Send(packetStr);
    }

    void send_packet();

    bool receive(std::vector<uint8_t>& buffer);

    inline bool is_connected() const{
        return m_connected;
    }
 
    inline void close()const{
        udp_bridge->Close();
    }

    inline void set_packet(telemetry_packet& ref_packet){
        std::lock_guard<std::mutex> lock(packet_mutex);
        packet = ref_packet;
    }

    bool validate_start_up(ground_station::start_up_packet& start_packet);

    inline void print_status(){
        std::cout<<"ipv4:" << ipv4_tel << "port :"<< port_tel <<"\n";
    }

    


};

}

#endif