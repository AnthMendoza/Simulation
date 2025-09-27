#ifndef VEHICLE_TELEMETRY_H
#define VEHICLE_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>




#define TELEMETRY_SYNC_WORD   0xABCD1234
#define TELEMETRY_VERSION     0x01


typedef enum : uint8_t {
    MSG_ATTITUDE   = 0x01,
    MSG_POSITION   = 0x02,
    MSG_VELOCITY   = 0x03,
    MSG_STATUS     = 0x04,
    MSG_ERROR      = 0x05
} telemetry_msg_type;


typedef enum : uint8_t {
    VEHICLE_INIT     = 0x00,
    VEHICLE_ARMED    = 0x01,
    VEHICLE_FLIGHT   = 0x02,
    VEHICLE_LANDING  = 0x03,
    VEHICLE_ERROR    = 0xFF
} vehicle_state;


typedef struct __attribute__((packed)) {
    uint32_t sync_word;       //sync word
    uint8_t  version;         //protocol version
    uint8_t  msg_type;        //message type
    uint16_t sequence;        //sequence number
    uint16_t payload_length;  //payload length
} telemetry_header;


typedef struct __attribute__((packed)) {
    float roll;
    float pitch;
    float yaw;
} attitude_data;

typedef struct __attribute__((packed)) {
    double latitude;
    double longitude;
    float altitude;
} position_data;

typedef struct __attribute__((packed)) {
    float vx;
    float vy;
    float vz;
} velocity_data;

typedef struct __attribute__((packed)) {
    vehicle_state state;
    uint32_t uptime;          // seconds
    uint8_t battery_percent;  // %
} status_data;

typedef struct __attribute__((packed)) {
    uint8_t code;
    char message[32];
} error_data;


typedef struct __attribute__((packed)){
    float angular_velocity; //rad/sec
}rotor_state;


typedef struct __attribute__((packed)) {
    telemetry_header header;
    union {
        attitude_data attitude;
        position_data position;
        velocity_data velocity;
        status_data   status;
        error_data    error;
        uint32_t      crc32;
    } payload;
} telemetry_packet;


                                
#define HEALTH_IMU_OK       (1 << 0)
#define HEALTH_GPS_OK       (1 << 1)
#define HEALTH_BARO_OK      (1 << 2)
#define HEALTH_MAG_OK       (1 << 3)
#define HEALTH_RADIO_OK     (1 << 4)
#define HEALTH_BATTERY_OK   (1 << 5)
#define HEALTH_MOTOR_OK     (1 << 6)
#define HEALTH_CONTROL_OK   (1 << 7)

#endif