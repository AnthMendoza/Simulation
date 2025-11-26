#pragma once

#include "../../ground_station/telemetry/include/packet_threaded.h"
#include "telemetry_packet.h"
#include "../third_party/CRC.h"
#include <variant>
#include <optional>

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


    void set_atitude(float roll, float pitch, float yaw) {
        attitude_data data{roll, pitch, yaw};
        write_packet(data, telemetry_msg_type::MSG_ATTITUDE);
    }
    void write_packet(const attitude_data& data, telemetry_msg_type type = telemetry_msg_type::MSG_ATTITUDE) {
        telemetry_packet packet;
        packet.header.msg_type = type;
        packet.payload.attitude = data;
        set_crc32(packet);
        set_packet(packet);
    }


    void set_position(float x, float y, float z) {
        position_data data{x, y, z};
        write_packet(data, telemetry_msg_type::MSG_POSITION);
    }
    void write_packet(const position_data& data, telemetry_msg_type type = telemetry_msg_type::MSG_POSITION) {
        telemetry_packet packet{};
        packet.header.msg_type = type;
        packet.payload.position = data;
        set_crc32(packet);
        set_packet(packet);
    }


    void set_velocity(float vx, float vy, float vz) {
        velocity_data data{vx, vy, vz};
        write_packet(data, telemetry_msg_type::MSG_VELOCITY);
    }
    void write_packet(const velocity_data& data, telemetry_msg_type type = telemetry_msg_type::MSG_VELOCITY) {
        telemetry_packet packet{};
        packet.header.msg_type = type;
        packet.payload.velocity = data;
        set_crc32(packet);
        set_packet(packet);
    }


    void set_status(vehicle_state state, uint32_t uptime, uint8_t battery_percent) {
        status_data data{state, uptime, battery_percent};
        write_packet(data, telemetry_msg_type::MSG_STATUS);
    }
    void write_packet(const status_data& data, telemetry_msg_type type = telemetry_msg_type::MSG_STATUS) {
        telemetry_packet packet{};
        packet.header.msg_type = type;
        packet.payload.status = data;
        set_crc32(packet);
        set_packet(packet);
    }    


    void set_error(uint8_t code, const std::string& msg) {
        error_data data{};
        data.code = code;
        strncpy(data.message, msg.c_str(), sizeof(data.message) - 1);
        write_packet(data, telemetry_msg_type::MSG_ERROR);
    }
    void write_packet(const error_data& data, telemetry_msg_type type = telemetry_msg_type::MSG_ERROR) {
        telemetry_packet packet{};
        packet.header.msg_type = type;
        packet.payload.error = data;
        set_crc32(packet);
        set_packet(packet);
    }
};