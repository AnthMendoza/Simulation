#ifndef PID_CONFIG_H
#define PID_CONFIG_H

#include <array>
#include <base/units.h>
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
    float attitude{0.0f};
    float rate{0.0f};
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
};

}
}


#endif