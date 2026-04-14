#ifndef PID_CONFIG_H
#define PID_CONFIG_H

#include <array>
#include <base/units.h>
#include <spdlog/spdlog.h>
#include <util/PIDController.h>


namespace avionics{
namespace pid{

struct p_con {
    float kp{0.0f};
    float i_limit{0.0f};
    float output_limit{0.0f};
};

struct position {
    p_con x;
    p_con y;
    p_con z;
};

struct velocity {
    pid_con x;
    pid_con y;
    pid_con z;
};


struct attitude {
    p_con roll;
    p_con pitch;
    p_con yaw;
};


struct rate {
    pid_con roll;
    pid_con pitch;
    pid_con yaw;
};


struct update_rates_hz {
    float position{0.0f};
    float velocity{0.0f};
    float acceleration{0.0f};
    float attitude{0.0f};
    float rate{0.0f};
    float telemetry{0.0f};
};

struct controller {
    std::string type;
    update_rates_hz update_rates_hz;
};

struct Limits {
    float max_tilt_angle_deg{0.0f};
    float max_ascent_rate_ms{0.0f};
    float max_descent_rate_ms{0.0f};
    float max_horizontal_vel_ms{0.0f};
    float motor_idle_throttle{0.0f};
    float motor_max_throttle{0.0f};
};


struct pid_config {
    controller controller;

    position position;
    velocity velocity;
    attitude attitude;
    rate rate;

    Limits limits;

    void print_debug(std::shared_ptr<spdlog::logger>& logger) {

    SPDLOG_LOGGER_DEBUG(logger, "Controller Type: {}", controller.type);
    SPDLOG_LOGGER_DEBUG(logger, "  Update Rates (Hz):");
    SPDLOG_LOGGER_DEBUG(logger, "    Position: {}", controller.update_rates_hz.position);
    SPDLOG_LOGGER_DEBUG(logger, "    Velocity: {}", controller.update_rates_hz.velocity);
    SPDLOG_LOGGER_DEBUG(logger, "    Acceleration:     {}", controller.update_rates_hz.acceleration);
    SPDLOG_LOGGER_DEBUG(logger, "    Attitude: {}", controller.update_rates_hz.attitude);
    SPDLOG_LOGGER_DEBUG(logger, "    Rate:     {}", controller.update_rates_hz.rate);
    SPDLOG_LOGGER_DEBUG(logger, "    Telemetry:     {}", controller.update_rates_hz.telemetry);

    SPDLOG_LOGGER_DEBUG(logger, "Position PIDs:");
    SPDLOG_LOGGER_DEBUG(logger, "  X: kp={}, i_limit={}, output_limit={}", position.x.kp, position.x.i_limit, position.x.output_limit);
    SPDLOG_LOGGER_DEBUG(logger, "  Y: kp={}, i_limit={}, output_limit={}", position.y.kp, position.y.i_limit, position.y.output_limit);
    SPDLOG_LOGGER_DEBUG(logger, "  Z: kp={}, i_limit={}, output_limit={}", position.z.kp, position.z.i_limit, position.z.output_limit);

    SPDLOG_LOGGER_DEBUG(logger, "Velocity PIDs:");
    SPDLOG_LOGGER_DEBUG(logger, "  X: kp={}, ki={}, kd={}, i_limit={}, output_limit={}", velocity.x.kp, velocity.x.ki, velocity.x.kd, velocity.x.i_limit, velocity.x.output_limit);
    SPDLOG_LOGGER_DEBUG(logger, "  Y: kp={}, ki={}, kd={}, i_limit={}, output_limit={}", velocity.y.kp, velocity.y.ki, velocity.y.kd, velocity.y.i_limit, velocity.y.output_limit);
    SPDLOG_LOGGER_DEBUG(logger, "  Z: kp={}, ki={}, kd={}, i_limit={}, output_limit={}", velocity.z.kp, velocity.z.ki, velocity.z.kd, velocity.z.i_limit, velocity.z.output_limit);

    SPDLOG_LOGGER_DEBUG(logger, "Attitude PIDs:");
    SPDLOG_LOGGER_DEBUG(logger, "  Roll:  kp={}, i_limit={}, output_limit={}", attitude.roll.kp,  attitude.roll.i_limit,  attitude.roll.output_limit);
    SPDLOG_LOGGER_DEBUG(logger, "  Pitch: kp={}, i_limit={}, output_limit={}", attitude.pitch.kp, attitude.pitch.i_limit, attitude.pitch.output_limit);
    SPDLOG_LOGGER_DEBUG(logger, "  Yaw:   kp={}, i_limit={}, output_limit={}", attitude.yaw.kp,   attitude.yaw.i_limit,   attitude.yaw.output_limit);

    SPDLOG_LOGGER_DEBUG(logger, "Rate PIDs:");
    SPDLOG_LOGGER_DEBUG(logger, "  Roll:  kp={}, ki={}, kd={}, i_limit={}, output_limit={}", rate.roll.kp,  rate.roll.ki,  rate.roll.kd,  rate.roll.i_limit,  rate.roll.output_limit);
    SPDLOG_LOGGER_DEBUG(logger, "  Pitch: kp={}, ki={}, kd={}, i_limit={}, output_limit={}", rate.pitch.kp, rate.pitch.ki, rate.pitch.kd, rate.pitch.i_limit, rate.pitch.output_limit);
    SPDLOG_LOGGER_DEBUG(logger, "  Yaw:   kp={}, ki={}, kd={}, i_limit={}, output_limit={}", rate.yaw.kp,   rate.yaw.ki,   rate.yaw.kd,   rate.yaw.i_limit,   rate.yaw.output_limit);

    SPDLOG_LOGGER_DEBUG(logger, "Limits:");
    SPDLOG_LOGGER_DEBUG(logger, "  Max Tilt Angle (deg):     {}", limits.max_tilt_angle_deg);
    SPDLOG_LOGGER_DEBUG(logger, "  Max Ascent Rate (m/s):    {}", limits.max_ascent_rate_ms);
    SPDLOG_LOGGER_DEBUG(logger, "  Max Descent Rate (m/s):   {}", limits.max_descent_rate_ms);
    SPDLOG_LOGGER_DEBUG(logger, "  Max Horizontal Vel (m/s): {}", limits.max_horizontal_vel_ms);
    SPDLOG_LOGGER_DEBUG(logger, "  Motor Idle Throttle:      {}", limits.motor_idle_throttle);
    SPDLOG_LOGGER_DEBUG(logger, "  Motor Max Throttle:       {}", limits.motor_max_throttle);
}
};

}
}


#endif