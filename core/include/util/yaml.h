#pragma once
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <string>


namespace utility{

template <typename T>
T getRequired(const YAML::Node& node, const std::string& key, const std::string& path = ""){
    const std::string fullPath = path.empty() ? key : path + "." + key;

    if(!node[key]){
        throw std::runtime_error("Missing field: " + fullPath);
    }

    try{
        return node[key].as<T>();
    }catch(const YAML::Exception& e){
        throw std::runtime_error("Invalid type for field: " + fullPath + " (" + e.what() + ")");
    }
}


template <typename T, std::size_t N>
std::array<T, N> getRequiredArr(const YAML::Node& node, const std::string& key,const std::string& path = "") {
    const std::string fullPath = path.empty() ? key : path + "." + key;
    if(!node[key]){
        throw std::runtime_error("Missing field: " + fullPath);
    }

    const YAML::Node& seq = node[key];

    if(!seq.IsSequence()){
        throw std::runtime_error("Expected a sequence for field: " + fullPath);
    }

    if (seq.size() != N){
        throw std::runtime_error("Size mismatch for field: " + fullPath + " (expected " + std::to_string(N) + ", got " + std::to_string(seq.size()) + ")");
    }

    try {
        std::array<T, N> result;
        for (std::size_t i = 0; i < N; ++i)
            result[i] = seq[i].as<T>();
        return result;
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Invalid element type in field: " + fullPath + " (" + e.what() + ")");
    }

}


}