#ifndef COMMUNICATION_BASE_H
#define COMMUNICATION_BASE_H 

#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <iostream>
#include <atomic>
#include "telemetry_packet.h"
#include "../../third_party/async-sockets/include/udpserver.hpp"
#include "../../../ground_station/telemetry/include/telemetry_ground.h"
#include "../../../ground_station/telemetry/include/packet_threaded.h"
#include <util/util.h>
#include "telemetry_packet_manager.h"
#include "../../include/scheduler_tasks.h"

namespace avionics{


class telemetry_base: public thread_manager{

private:

scheduler telemetry_scheduler;
std::shared_ptr<UDPServer<>> udp_bridge;
std::string m_interface;

std::string ipv4_tel;
uint16_t port_tel;
int m_baudrate;

void set_packet_from_callback(){
   
}

protected:

bool m_connected;
bool relay_connected_to_ground;

public:
    telemetry_packet_manager packet_manager;

    //note baudRate does not define the rate at which packets are sent. 
    telemetry_base();


    ~telemetry_base() = default;

    void initialize_base(){
       std::vector<scheduler_tasks> tasks = get_routine();
       for (auto& task : tasks){
           telemetry_scheduler.add_manager(task.interval_s,task.callback);
       }
       set_scheduler(telemetry_scheduler);
    }

    virtual std::vector<scheduler_tasks> get_routine() = 0;

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
        packet_manager.set_packet(ref_packet);
    }

    bool validate_start_up(ground_station::start_up_packet& start_packet);

    inline void print_status(){
        std::cout<<"ipv4:" << ipv4_tel << "port :"<< port_tel <<"\n";
    }
    using t_packet = avionics::flight_controller_telemetry_packet;

    std::function<t_packet(float)> set_telemetry_callback;

    void set_telemetry_buffer(std::function<t_packet(float)> callback){
        set_telemetry_callback = callback;
    }
    


};

}

#endif