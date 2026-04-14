#include "../include/pid_config_parse.hpp"

namespace avionics{
namespace pid{


static p_con load_p_con(const YAML::Node& node, const std::string& path){
    if (!node) throw std::runtime_error("Missing PID node: " + path);
    p_con out;
    if (!node["kp"])           throw std::runtime_error("Missing field: " + path + ".kp");
    if (!node["i_limit"])      throw std::runtime_error("Missing field: " + path + ".i_limit");
    if (!node["output_limit"]) throw std::runtime_error("Missing field: " + path + ".output_limit");
    out.kp           = node["kp"].as<float>();
    out.i_limit      = node["i_limit"].as<float>();
    out.output_limit = node["output_limit"].as<float>();
    return out;
}
static pid_con load_pid_con(const YAML::Node& node, const std::string& path){
    if (!node) throw std::runtime_error("Missing PID node: " + path);
    pid_con out;
    if (!node["kp"])           throw std::runtime_error("Missing field: " + path + ".kp");
    if (!node["ki"])           throw std::runtime_error("Missing field: " + path + ".ki");
    if (!node["kd"])           throw std::runtime_error("Missing field: " + path + ".kd");
    if (!node["i_limit"])      throw std::runtime_error("Missing field: " + path + ".i_limit");
    if (!node["output_limit"]) throw std::runtime_error("Missing field: " + path + ".output_limit");
    out.kp           = node["kp"].as<float>();
    out.ki           = node["ki"].as<float>();
    out.kd           = node["kd"].as<float>();
    out.i_limit      = node["i_limit"].as<float>();
    out.output_limit = node["output_limit"].as<float>();
    return out;
}
pid_config load_pid_config(const std::string& filepath){
    YAML::Node root = YAML::LoadFile(filepath);
    pid_config config;


    const auto& ctrl = root["controller"];
    if (!ctrl) throw std::runtime_error("Missing section: controller");
    if (!ctrl["type"]) throw std::runtime_error("Missing field: controller.type");
    config.controller.type = ctrl["type"].as<std::string>();

    const auto& rates = ctrl["update_rates_hz"];
    if (!rates) throw std::runtime_error("Missing section: controller.update_rates_hz");
    if (!rates["position"]) throw std::runtime_error("Missing field: controller.update_rates_hz.position");
    if (!rates["velocity"]) throw std::runtime_error("Missing field: controller.update_rates_hz.velocity");
    if (!rates["attitude"]) throw std::runtime_error("Missing field: controller.update_rates_hz.attitude");
    if (!rates["rate"])     throw std::runtime_error("Missing field: controller.update_rates_hz.rate");
    config.controller.update_rates_hz.position = rates["position"].as<float>();
    config.controller.update_rates_hz.velocity = rates["velocity"].as<float>();
    config.controller.update_rates_hz.attitude = rates["attitude"].as<float>();
    config.controller.update_rates_hz.rate     = rates["rate"].as<float>();

    
    const auto& pos = root["position"];
    if (!pos) throw std::runtime_error("Missing section: position");
    config.position.x = load_p_con(pos["x"], "position.x");
    config.position.y = load_p_con(pos["y"], "position.y");
    config.position.z = load_p_con(pos["z"], "position.z");

    

    const auto& vel = root["velocity"];
    if (!vel) throw std::runtime_error("Missing section: velocity");
    config.velocity.x = load_pid_con(vel["x"], "velocity.x");
    config.velocity.y = load_pid_con(vel["y"], "velocity.y");
    config.velocity.z = load_pid_con(vel["z"], "velocity.z");


    const auto& att = root["attitude"];
    if (!att) throw std::runtime_error("Missing section: attitude");
    config.attitude.roll  = load_p_con(att["roll"],  "attitude.roll");
    config.attitude.pitch = load_p_con(att["pitch"], "attitude.pitch");
    config.attitude.yaw   = load_p_con(att["yaw"],   "attitude.yaw");

  
    const auto& rate = root["rate"];
    if(!rate) throw std::runtime_error("Missing section: rate");
    config.rate.roll  = load_pid_con(rate["roll"],  "rate.roll");
    config.rate.pitch = load_pid_con(rate["pitch"], "rate.pitch");
    config.rate.yaw   = load_pid_con(rate["yaw"],   "rate.yaw");


    const auto& lim = root["limits"];
    if(!lim) throw std::runtime_error("Missing section: limits");
    if(!lim["max_tilt_angle_deg"])    throw std::runtime_error("Missing field: limits.max_tilt_angle_deg");
    if(!lim["max_ascent_rate_ms"])    throw std::runtime_error("Missing field: limits.max_ascent_rate_ms");
    if(!lim["max_descent_rate_ms"])   throw std::runtime_error("Missing field: limits.max_descent_rate_ms");
    if(!lim["max_horizontal_vel_ms"]) throw std::runtime_error("Missing field: limits.max_horizontal_vel_ms");
    if(!lim["motor_idle_throttle"])   throw std::runtime_error("Missing field: limits.motor_idle_throttle");
    if(!lim["motor_max_throttle"])    throw std::runtime_error("Missing field: limits.motor_max_throttle");
    config.limits.max_tilt_angle_deg    = lim["max_tilt_angle_deg"].as<float>();
    config.limits.max_ascent_rate_ms    = lim["max_ascent_rate_ms"].as<float>();
    config.limits.max_descent_rate_ms   = lim["max_descent_rate_ms"].as<float>();
    config.limits.max_horizontal_vel_ms = lim["max_horizontal_vel_ms"].as<float>();
    config.limits.motor_idle_throttle   = lim["motor_idle_throttle"].as<float>();
    config.limits.motor_max_throttle    = lim["motor_max_throttle"].as<float>();

    return config;
}

}

}
