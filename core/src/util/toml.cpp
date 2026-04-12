#include "../../include/util/toml.h"
#include <sstream>

namespace toml {

namespace {

std::string trim(const std::string& input) {
    const auto first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }

    const auto last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1);
}

std::string stripComment(const std::string& line) {
    bool inQuotes = false;
    for (size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '"') {
            inQuotes = !inQuotes;
        }
        if (!inQuotes && line[i] == '#') {
            return trim(line.substr(0, i));
        }
    }
    return trim(line);
}

void parseKeyValueLine(
    const std::string& line,
    std::unordered_map<std::string, float>& floatValues,
    std::unordered_map<std::string, bool>& boolValues,
    std::unordered_map<std::string, std::vector<float>>& arrayValues,
    std::unordered_map<std::string, std::string>& mapOfStrings
) {
    // Shared parser for both:
    // [section]
    // key = value
    //
    // and repeated tables:
    // [[actuator]]
    // position = [0.1, 0.2, 0.0]
    const auto equalsPos = line.find('=');
    if (equalsPos == std::string::npos) {
        return;
    }

    std::string key = trim(line.substr(0, equalsPos));
    std::string value = trim(line.substr(equalsPos + 1));
    if (value.empty()) {
        return;
    }

    if (value.front() == '[' && value.back() == ']') {
        value = value.substr(1, value.size() - 2);
        std::istringstream arrStream(value);
        std::string numStr;
        std::vector<float> values;

        while (std::getline(arrStream, numStr, ',')) {
            numStr = trim(numStr);
            if (!numStr.empty()) {
                values.push_back(std::stof(numStr));
            }
        }
        arrayValues[key] = values;
    } else if (value == "false") {
        boolValues[key] = false;
    } else if (value == "true") {
        boolValues[key] = true;
    } else if (value.front() == '"' && value.back() == '"') {
        mapOfStrings[key] = value.substr(1, value.size() - 2);
    } else {
        try {
            floatValues[key] = std::stof(value);
        } catch (const std::invalid_argument&) {
            mapOfStrings[key] = value;
        }
    }
}

}

tomlParse::tomlParse(){

}

void tomlParse::parseConfig(const std::string& config, const std::string& targetSection) {
    floatValues.clear();
    boolValues.clear();
    arrayValues.clear();
    mapOfStrings.clear();

    std::istringstream stream(config);
    std::string line;
    bool inSection = false;

    while (std::getline(stream, line)) {
        line = stripComment(line);
        if (line.empty()) continue;

        if (line[0] == '[') {
            inSection = (line == "[" + targetSection + "]");
            continue;
        }

        if (!inSection) continue;

        parseKeyValueLine(line, floatValues, boolValues, arrayValues, mapOfStrings);
    }
}

std::vector<tomlParse> tomlParse::parseArrayOfTables(
    const std::string& config,
    const std::string& targetSection
) {
    // Example targetSection = "actuator" will read:
    // [[actuator]]
    // name = "front_right"
    //
    // [[actuator]]
    // name = "front_left"
    std::vector<tomlParse> tables;
    std::istringstream stream(config);
    std::string line;
    tomlParse* currentTable = nullptr;

    while (std::getline(stream, line)) {
        line = stripComment(line);
        if (line.empty()) continue;

        if (line == "[[" + targetSection + "]]") {
            tables.emplace_back();
            currentTable = &tables.back();
            continue;
        }

        if (line.front() == '[') {
            currentTable = nullptr;
            continue;
        }

        if (currentTable == nullptr) {
            continue;
        }

        parseKeyValueLine(
            line,
            currentTable->floatValues,
            currentTable->boolValues,
            currentTable->arrayValues,
            currentTable->mapOfStrings
        );
    }

    return tables;
}

}
