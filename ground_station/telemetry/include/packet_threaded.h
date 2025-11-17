#pragma once
#include <mutex>
#include "../../../flight_computer/telemetry/include/telemetry_packet.h"
#include <memory>

namespace telemetry{

template<typename packet_type>

class packet_with_mutex{
    private:

    std::mutex packet_mutex;

    std::shared_ptr<packet_type> packet;

    public:

    packet_with_mutex(){
        packet_type packet_empty = {};
        packet = std::make_shared<packet_type>(packet_empty);
    }

    packet_type get_packet(){
        std::lock_guard<std::mutex> lock(packet_mutex);
        return *packet;
    }

    void set_packet(packet_type ref_packet){
        std::lock_guard<std::mutex> lock(packet_mutex);
        *packet = ref_packet;
    }

    void set_packet_raw_data(const char *data , size_t length){
        if(length != sizeof(packet_type)){
            return;
        }
        std::lock_guard<std::mutex> lock(packet_mutex);
        std::memcpy(packet.get(), data, sizeof(packet_type));
    }

};

}