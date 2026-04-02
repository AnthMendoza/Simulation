#ifndef TOML_H
#define TOML_H
#include <unordered_map>
#include <vector>
#include <string>
#include <stdexcept>

namespace toml {
class tomlParse {

public:
    tomlParse();
    std::unordered_map<std::string, float> floatValues;
    std::unordered_map<std::string, bool> boolValues;
    std::unordered_map<std::string, std::vector<float>> arrayValues;
    std::unordered_map<std::string,std::string> mapOfStrings;
    void parseConfig(const std::string& config, const std::string& targetSection);
    inline float getFloat(const std::string& key) const {
        auto value = floatValues.find(key);
        if (value == floatValues.end()) {
            throw std::runtime_error("Key '" + key + "' not found in floatValues");
        }
        return value->second;
    }

    inline bool getBool(const std::string& key) const {
        auto value = boolValues.find(key);
        if (value == boolValues.end()) {
            throw std::runtime_error("Key '" + key + "' not found in boolValues");
        }
        return value->second;
    }

    inline std::vector<float> getArray(const std::string& key) const {
        auto value = arrayValues.find(key);
        if (value == arrayValues.end()) {
            throw std::runtime_error("Key '" + key + "' not found in arrayValues");
        }
        return value->second;
    }

    inline std::string getString(const std::string& key) const{
        auto value = mapOfStrings.find(key);
        if(value == mapOfStrings.end()){
            throw std::runtime_error("Key '" + key + "' not found in getString");
        }
        return value->second;
    }
};
}
#endif

