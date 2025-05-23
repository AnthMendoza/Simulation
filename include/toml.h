#ifndef TOML_H
#define TOML_H
#include <unordered_map>
#include <vector>
#include <string>

namespace toml {
class tomlParse {

public:
    tomlParse();
    std::unordered_map<std::string, float> floatValues;
    std::unordered_map<std::string, bool> boolValues;
    std::unordered_map<std::string, std::vector<float>> arrayValues;
    void parseConfig(const std::string& config, const std::string& targetSection);
};
}
#endif

