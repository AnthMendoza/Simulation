#include "../include/airframe_config.h"

#include <stdexcept>

#include <util/toml.h>
#include <util/utility.h>

namespace {

std::string resolvePath(const std::string& rawPath, const std::filesystem::path& baseDirectory) {
    if (rawPath.empty()) {
        return "";
    }

    const std::filesystem::path path(rawPath);
    if (path.is_absolute() || baseDirectory.empty()) {
        return path.lexically_normal().string();
    }

    return (baseDirectory / path).lexically_normal().string();
}

void validateActuator(const avionics::actuator_config& actuator, size_t index) {
    if (actuator.name.empty()) {
        throw std::runtime_error("Actuator #" + std::to_string(index) + " is missing a name");
    }
    if (actuator.rotationDirection != -1 && actuator.rotationDirection != 1) {
        throw std::runtime_error(
            "Actuator '" + actuator.name + "' rotationDirection must be either -1 or 1"
        );
    }
    if (actuator.motorConfigPath.empty()) {
        throw std::runtime_error("Actuator '" + actuator.name + "' is missing a motorConfig path");
    }
    if (actuator.propellerConfigPath.empty()) {
        throw std::runtime_error(
            "Actuator '" + actuator.name + "' is missing a propellerConfig path"
        );
    }
}

}

namespace avionics{

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

airframe_config airframe_config_parser::fromToml(
    const std::string& configText,
    const std::filesystem::path& baseDirectory
) {
    airframe_config config;
    toml::tomlParse rootParser;
    rootParser.parseConfig(configText, "component_library");

    if (rootParser.mapOfStrings.contains("motor")) {
        config.defaultMotorConfigPath = resolvePath(rootParser.getString("motor"), baseDirectory);
    }
    if (rootParser.mapOfStrings.contains("propeller")) {
        config.defaultPropellerConfigPath = resolvePath(rootParser.getString("propeller"), baseDirectory);
    }

    const auto actuatorTables = rootParser.parseArrayOfTables(configText, "actuator");
    config.actuators.reserve(actuatorTables.size());

    for (const auto& table : actuatorTables) {
        actuator_config actuator;
        if (table.mapOfStrings.contains("name")) {
            actuator.name = table.getString("name");
        }
        if (table.arrayValues.contains("position")) {
            const auto& position = table.arrayValues.at("position");
            if (position.size() != 3) {
                throw std::runtime_error("Actuator '" + actuator.name + "' position must have 3 elements");
            }
            actuator.position = {position[0], position[1], position[2]};
        }
        if (table.arrayValues.contains("thrustDirection")) {
            const auto& direction = table.arrayValues.at("thrustDirection");
            if (direction.size() != 3) {
                throw std::runtime_error("Actuator '" + actuator.name + "' thrustDirection must have 3 elements");
            }
            actuator.thrustDirection = {direction[0], direction[1], direction[2]};
        }
        if (table.floatValues.contains("rotationDirection")) {
            actuator.rotationDirection = static_cast<int>(table.getFloat("rotationDirection"));
        }
        if (table.mapOfStrings.contains("motorConfig")) {
            actuator.motorConfigPath = resolvePath(table.getString("motorConfig"), baseDirectory);
        } else if (table.mapOfStrings.contains("motor")) {
            actuator.motorConfigPath = resolvePath(table.getString("motor"), baseDirectory);
        }
        if (table.mapOfStrings.contains("propellerConfig")) {
            actuator.propellerConfigPath = resolvePath(table.getString("propellerConfig"), baseDirectory);
        } else if (table.mapOfStrings.contains("propeller")) {
            actuator.propellerConfigPath = resolvePath(table.getString("propeller"), baseDirectory);
        }
        config.actuators.push_back(actuator);
    }

    if (config.actuators.empty()) {
        throw std::runtime_error("No [[actuator]] entries were found in the airframe config");
    }

    for (size_t i = 0; i < config.actuators.size(); ++i) {
        auto& actuator = config.actuators[i];
        if (actuator.motorConfigPath.empty()) {
            actuator.motorConfigPath = config.defaultMotorConfigPath;
        }
        if (actuator.propellerConfigPath.empty()) {
            actuator.propellerConfigPath = config.defaultPropellerConfigPath;
        }
        validateActuator(actuator, i);
    }

    return config;
}

airframe_config airframe_config_parser::fromFile(const std::string& filePath) {
    const std::filesystem::path sourcePath(filePath);
    const std::string contents = utility::readFileAsString(filePath);
    if (contents.empty()) {
        throw std::runtime_error("Unable to read airframe config file: " + filePath);
    }

    airframe_config config = fromToml(contents, sourcePath.parent_path());
    config.sourcePath = sourcePath.lexically_normal();
    return config;
}

}  // namespace avionics
