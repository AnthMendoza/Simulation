#ifndef AIRFRAME_CONFIG_H
#define AIRFRAME_CONFIG_H

#include <array>
#include <filesystem>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>
#include <base/units.h>
#include <yaml-cpp/yaml.h>

namespace avionics{

struct actuator_config {
    std::string name;
    units::vec3 position = {0.0f, 0.0f, 0.0f};
    units::vec3 thrustDirection = {0.0f, 0.0f, 1.0f};
    int rotationDirection = 1;
    std::string motorConfigPath;
    std::string propellerConfigPath;
    
    void print_debug(std::shared_ptr<spdlog::logger>& logger){

        SPDLOG_LOGGER_DEBUG(logger,"Actuator: {}", name);
        SPDLOG_LOGGER_DEBUG(logger,"  Position: [{}, {}, {}]",
                     position[0], position[1], position[2]);
        SPDLOG_LOGGER_DEBUG(logger,"  Thrust Direction: [{}, {}, {}]",
                     thrustDirection[0], thrustDirection[1], thrustDirection[2]);
        SPDLOG_LOGGER_DEBUG(logger,"  Rotation Direction: {}", rotationDirection);
        SPDLOG_LOGGER_DEBUG(logger,"  Motor Config Path: {}", motorConfigPath);
        SPDLOG_LOGGER_DEBUG(logger,"  Propeller Config Path: {}", propellerConfigPath);

    }
};

struct environment{    
    units::scalar gravitationalAcceleration = 9.8067f;
};


struct airframe_config {
    std::filesystem::path sourcePath;
    std::string defaultMotorConfigPath;
    std::string defaultPropellerConfigPath;
    std::vector<actuator_config> actuators;
    environment env;

    std::vector<units::vec3> motorPositions()const;
    std::vector<units::vec3> thrustDirections()const;
    std::vector<units::scalar> rotationDirections()const ;

    void print_debug(std::shared_ptr<spdlog::logger>& logger){

        SPDLOG_LOGGER_DEBUG(logger,"Airframe Config:");
        SPDLOG_LOGGER_DEBUG(logger,"  Source Path: {}", sourcePath.string());
        SPDLOG_LOGGER_DEBUG(logger,"  Default Motor Config: {}", defaultMotorConfigPath);
        SPDLOG_LOGGER_DEBUG(logger,"  Default Propeller Config: {}", defaultPropellerConfigPath);
        SPDLOG_LOGGER_DEBUG(logger,"  Number of Actuators: {}", actuators.size());

        for (size_t i = 0; i < actuators.size(); ++i) {
            SPDLOG_LOGGER_DEBUG(logger,"  Actuator [{}]:", i);
            actuators[i].print_debug(logger); 
        }
    }
};

class airframe_config_parser{
private:
    
    static void v_path(airframe_config& config , YAML::Node& node);

public:

    static airframe_config fromFile(const std::string& filePath);


};

}

#endif
