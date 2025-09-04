#include "../../include/sim/toml.h"
#include <sstream>

namespace toml {
tomlParse::tomlParse(){

}

void tomlParse::parseConfig(const std::string& config, const std::string& targetSection) {
    std::istringstream stream(config);
    std::string line;
    bool inSection = false;

    while (std::getline(stream, line)) {
        line.erase(0, line.find_first_not_of(" \t")); 
        if (line.empty() || line[0] == '#') continue;

        if (line[0] == '[') {
            inSection = (line == "[" + targetSection + "]");
            continue;
        }

        if (!inSection) continue;

        auto equalsPos = line.find('=');
        if (equalsPos == std::string::npos) continue;

        std::string key = line.substr(0, equalsPos);
        std::string value = line.substr(equalsPos + 1);
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));

        // Array
        if (value.front() == '[' && value.back() == ']') {
            value = value.substr(1, value.size() - 2); 
            std::istringstream arrStream(value);
            std::string numStr;
            std::vector<float> values;

            while (std::getline(arrStream, numStr, ',')) {
                numStr.erase(0, numStr.find_first_not_of(" \t"));
                values.push_back(std::stof(numStr));
            }
            arrayValues[key] = values;
        }
        // Bool
        else if (value == "false") {
            boolValues[key] = false;
        }
        else if (value == "true") {
            boolValues[key] = true;
        }

        else if (value.front() == '"' && value.back() == '"') {
            mapOfStrings[key] = value.substr(1, value.size() - 2);
        }

        else {
            try {
                floatValues[key] = std::stof(value);
            } catch (const std::invalid_argument&) {
                mapOfStrings[key] = value;
            }
        }
    }
}

}
