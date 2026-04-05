#pragma once

#include <util/thread_manager.h>
#include <async-sockets/include/udpsocket.hpp>
#include <telemetry_packet.h>
#include "../../telemetry/include/packet_threaded.h"

namespace gui_connector{

using packet_mutex_ptr = std::shared_ptr<utility::telemetry::packet_with_mutex<avionics::telemetry_packet_gui>>;

class gui_connector : public thread_manager{

private:
    std::shared_ptr<UDPSocket<>> udp_bridge;
    packet_mutex_ptr packet_ptr;
    uint16_t port_tel;
public:

    gui_connector(float packet_rate);

    virtual void thread_process() override;

    virtual void thread_startup_process() override;

    void send_packet();

    void call_back();

    template<typename packetType>
    void send(const packetType& packet) {
        const char* rawData = reinterpret_cast<const char*>(&packet);

        std::string packetStr(rawData, sizeof(packetType));

        udp_bridge->Send(packetStr);
    }

    void set_packet_ptr(packet_mutex_ptr ref_ptr){
        packet_ptr = ref_ptr;
    }

};

}