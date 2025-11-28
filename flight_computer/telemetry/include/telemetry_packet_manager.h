#pragma once

#include "../../../ground_station/telemetry/include/packet_threaded.h"
#include "telemetry_packet.h"
#include "../../third_party/CRC.h"
#include <variant>
#include <optional>
#include <mutex>


class telemetry_packet_manager : public utility::telemetry::packet_with_mutex<telemetry_packet>{

private: 

    void set_crc32(telemetry_packet& packet){
        
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&packet);
        //excludes the crc32 uint32 from the caluclation
        size_t length = sizeof(packet) - sizeof(packet.crc32);

        packet.crc32 = CRC::Calculate(bytes, length , CRC::CRC_32());

    }


    bool check_crc32(const telemetry_packet& packet){

        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&packet);

        size_t length = sizeof(packet) - sizeof(packet.crc32);
        
        uint32_t CRC_ref = CRC::Calculate(bytes,length,CRC::CRC_32());

        if(CRC_ref == packet.crc32){
            return true;
        }

        return false;
    }


public:
    packet_with_mutex<flight_controller_telemetry_packet> flight_controller_packet;

    using PayloadVariant = std::variant<
        attitude_data,
        position_data,
        velocity_data,
        status_data,
        error_data
    >;
    
    //return nullopt if failed
    std::optional<PayloadVariant> recieved(const uint8_t* buffer, size_t size){
        if(size != sizeof(telemetry_packet)){
            return std::nullopt;
        }

        telemetry_packet packet;
        memcpy(&packet, buffer, sizeof(telemetry_packet));

        if (!check_crc32(packet)) {
            return std::nullopt;
        }

        switch (packet.header.msg_type) {
            case 0x01: return packet.payload.attitude;
            case 0x02: return packet.payload.position;
            case 0x03: return packet.payload.velocity;
            case 0x04: return packet.payload.status;
            case 0x05: return packet.payload.error;
            default:   return std::nullopt;
        }

    }



    #define DEFINE_PACKET_BUILDER(TYPE, FIELD) \
    telemetry_packet get_##TYPE##_packet() { \
        telemetry_packet packet = {}; \
        packet.header.msg_type = telemetry_msg_type::MSG_##TYPE; \
        flight_controller_packet.modify([&packet](flight_controller_telemetry_packet& src){ \
            packet.payload.FIELD = src.FIELD; \
        }); \
        set_crc32(packet);\
        return packet; \
    }

    DEFINE_PACKET_BUILDER(ATTITUDE, attitude)
    DEFINE_PACKET_BUILDER(POSITION, position)
    DEFINE_PACKET_BUILDER(VELOCITY, velocity)
    DEFINE_PACKET_BUILDER(STATUS, status)
    DEFINE_PACKET_BUILDER(ERROR, error)
    
    #undef DEFINE_PACKET_BUILDER
};