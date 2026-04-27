#pragma once
#include <cstdint>
#include <atomic>

namespace avionics{

enum class flight_mode : uint8_t{
    boot,
    standby,
    armed,
    flight,
    safe,
    fault
};

using lc_type = uint8_t;

enum class subsystem_lifecycle : lc_type {
    lifecycle_none        = 0,
    lifecycle_boot        = 1 << 0,
    lifecycle_initialized = 1 << 1,
    lifecycle_running     = 1 << 2,
    lifecycle_fault       = 1 << 3,
    lifecycle_offline     = 1 << 4,
    lifecycle_connecting  = 1 << 5, //connecting and connected will not apply to all subsystems
    lifecycle_connected   = 1 << 6
};

using h_type = uint8_t;

enum class health_status : h_type {
    health_nominal  = 1 << 0,
    health_warning  = 1 << 1,
    health_critical = 1 << 2
};

struct subsystem_health {
    std::atomic<lc_type> lifecycle {static_cast<lc_type>(subsystem_lifecycle::lifecycle_boot)};
    std::atomic<h_type> status {static_cast<h_type>(health_status::health_nominal)};

    std::atomic<uint64_t> last_update_us {0};
    std::atomic<uint32_t> error_count {0};
};

enum class fault_flags : uint32_t{
    hardware_failure      = 1 << 0,
    estimator_failure     = 1 << 1,
    controller_failure    = 1 << 2,
    communication_failure = 1 << 3,

    sensor_timeout        = 1 << 4,
    watchdog_triggered    = 1 << 5,
    low_power             = 1 << 6,

    critical              = 1 << 7
};

struct flight_computer_health{
    std::atomic<flight_mode> mode{flight_mode::boot};

    subsystem_health hardware;
    subsystem_health estimator;
    subsystem_health controller;
    subsystem_health communication;

    std::atomic<fault_flags> faults;

};

} // namespace avionics