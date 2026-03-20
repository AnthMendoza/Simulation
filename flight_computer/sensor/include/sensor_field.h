#ifndef SENSOR_FEILD_H
#define SENSOR_FEILD_H


#include <cstdint>
#include <array>
#include <optional>

namespace avionics::sensor{

// ─── Coordinate frame / mounting metadata ────────────────────────────────────
struct SensorMount {
    std::array<float, 3> position_m;        // offset from body CoM [x, y, z] in meters
    std::array<float, 4> orientation_q;     // quaternion [w, x, y, z] relative to body frame
};

// ─── Per-sample health / validity flags ──────────────────────────────────────
struct SensorStatus {
    bool data_valid      : 1;
    bool saturated       : 1;
    bool self_test_pass  : 1;
    bool calibrated      : 1;
    uint8_t health_code;                    // device-specific error register
};

// ─── IMU / Accelerometer ─────────────────────────────────────────────────────
struct AccelData {
    // --- raw output ---
    std::array<float, 3> accel_mss;         // [ax, ay, az] m/s²
    float temperature_c;                    // on-die temperature

    // --- state-estimation metadata ---
    std::array<float, 3> bias_mss;          // estimated bias [ax, ay, az]
    std::array<float, 9> noise_cov;         // 3×3 measurement noise covariance (row-major)
    float scale_factor;                     // gain calibration scalar
    uint64_t timestamp_us;                  // capture time (µs, monotonic clock)
    uint32_t sample_rate_hz;

    SensorMount  mount;
    SensorStatus status;
};

// ─── Gyroscope ───────────────────────────────────────────────────────────────
struct GyroData {
    // --- raw output ---
    std::array<float, 3> gyro_rads;         // [p, q, r] rad/s

    // --- state-estimation metadata ---
    std::array<float, 3> bias_rads;
    std::array<float, 9> noise_cov;
    float scale_factor;
    uint64_t timestamp_us;
    uint32_t sample_rate_hz;

    SensorMount  mount;
    SensorStatus status;
};

// ─── Barometer ───────────────────────────────────────────────────────────────
struct BaroData {
    // --- raw output ---
    float pressure_pa;
    float altitude_m;                       // derived via ISA model
    float temperature_c;

    // --- state-estimation metadata ---
    float noise_variance;                   // σ² for altitude measurement
    float bias_m;                           // estimated altitude bias
    uint64_t timestamp_us;
    uint32_t sample_rate_hz;

    SensorMount  mount;
    SensorStatus status;
};

// ─── Magnetometer ────────────────────────────────────────────────────────────
struct MagData {
    // --- raw output ---
    std::array<float, 3> mag_gauss;         // [mx, my, mz] Gauss

    // --- state-estimation metadata ---
    std::array<float, 9> soft_iron;         // 3×3 soft-iron correction (row-major)
    std::array<float, 3> hard_iron_bias;    // hard-iron offset
    std::array<float, 9> noise_cov;
    uint64_t timestamp_us;
    uint32_t sample_rate_hz;

    SensorMount  mount;
    SensorStatus status;
};

// ─── GPS / GNSS ──────────────────────────────────────────────────────────────
struct GpsData {
    // --- raw output ---
    double  latitude_deg;
    double  longitude_deg;
    float   altitude_msl_m;
    std::array<float, 3> velocity_ned_ms;   // North-East-Down velocity

    // --- state-estimation metadata ---
    std::array<float, 9> pos_cov_m2;        // 3×3 position covariance [m²]
    std::array<float, 9> vel_cov_m2s2;      // 3×3 velocity covariance [m²/s²]
    uint8_t  num_satellites;
    float    hdop;
    uint8_t  fix_type;                      // 0=none,1=2D,2=3D,3=RTK-float,4=RTK-fixed
    uint64_t timestamp_us;
    uint32_t sample_rate_hz;

    SensorMount  mount;
    SensorStatus status;
};

// ─── Optical Flow ─────────────────────────────────────────────────────────────
struct OptFlowData {
    // --- raw output ---
    std::array<float, 2> flow_rads;         // [flow_x, flow_y] rad/s
    float     ground_distance_m;            // rangefinder reading
    uint8_t   quality;                      // 0–255 feature-match quality

    // --- state-estimation metadata ---
    float     noise_variance;
    uint64_t  timestamp_us;
    uint32_t  sample_rate_hz;

    SensorMount  mount;
    SensorStatus status;
};

// ─── Rangefinder / LiDAR ─────────────────────────────────────────────────────
struct RangeData {
    // --- raw output ---
    float range_m;
    float signal_strength;

    // --- state-estimation metadata ---
    float noise_variance;
    float min_range_m;
    float max_range_m;
    uint64_t timestamp_us;
    uint32_t sample_rate_hz;

    SensorMount  mount;
    SensorStatus status;
};


template<typename T, std::size_t N>
using SensorArray = std::array<std::optional<T>, N>;

struct SensorField {
    SensorArray<AccelData, 1> accel;
    SensorArray<GyroData,  1> gyro;

    SensorArray<BaroData,  1> baro;
    SensorArray<MagData,   1> mag;
    SensorArray<GpsData,   1> gps; 

    SensorArray<OptFlowData, 1> opt_flow;
    SensorArray<RangeData,   1> rangefinder; 
};

}
#endif