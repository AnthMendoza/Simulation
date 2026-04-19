#include "../include/airframe_config.h"

#include <stdexcept>
#include <util/toml.h>
#include <util/utility.h>
#include <util/yaml.h>
#include <yaml-cpp/yaml.h>





namespace avionics{


airframe_config airframe_config_parser::fromFile(const std::string& filePath){
    YAML::Node node = YAML::LoadFile(filePath);
    airframe_config config;

    const auto& actuators = node["actuator"];

    if(!actuators.IsSequence()){
        throw std::runtime_error("'actuator' must be a sequence");
    }
    config.actuators.resize(actuators.size());
    for(int i = 0 ; i < actuators.size() ; i++){
        const auto& a_node = actuators[i];
        auto& act = config.actuators[i];
        std::string path = "actuator number : " + std::to_string(i);
        act.name = utility::getRequired<std::string>(a_node,"name",path);
        act.position = utility::getRequiredArr<units::scalar,3>(a_node,"position",path);
        act.thrustDirection = utility::getRequiredArr<units::scalar,3>(a_node,"thrustDirection",path);
        act.rotationDirection = utility::getRequired<units::scalar>(a_node,"rotationDirection",path);
    }

    const auto& components = node["component_library"];

    config.defaultMotorConfigPath = utility::getRequired<std::string>(components,"motor","Motor default path");
    config.defaultPropellerConfigPath = utility::getRequired<std::string>(components,"propeller","Propeller default path");

    return config;

}



std::vector<std::array<float, 3>> airframe_config::motorPositions() const {
    std::vector<std::array<float, 3>> positions;
    positions.reserve(actuators.size());

    for (const auto& actuator : actuators) {
        positions.push_back(actuator.position);
    }

    return positions;
}

std::vector<std::array<float, 3>> airframe_config::thrustDirections() const {
    std::vector<std::array<float, 3>> directions;
    directions.reserve(actuators.size());

    for (const auto& actuator : actuators) {
        directions.push_back(actuator.thrustDirection);
    }

    return directions;
}

std::vector<float> airframe_config::rotationDirections() const {
    std::vector<float> directions;
    directions.reserve(actuators.size());

    for (const auto& actuator : actuators) {
        directions.push_back(static_cast<float>(actuator.rotationDirection));
    }

    return directions;
}

}
