#ifndef DRONELOGGING_H
#define DRONELOGGING_H
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <unordered_map>
#include <map>
#include <memory>
#include <iostream>

namespace SimCore{
namespace log{
//#define DRONE_LOGGING

#ifdef DRONE_LOGGING
    #define LOG_SCALAR(channel, time, value) \
        if (SimCore::log::logger) { \
            SimCore::log::logger->logData(channel, time, value); \
        }
    #define LOG_VEC3(channel, time, x, y, z) \
        if (SimCore::log::logger) { \
            SimCore::log::logger->logData(channel, time, std::array<float, 3>{x, y, z}); \
        }
#else
    //avoids warnings if called when disabled. prvents the need for more #ifdef
    #define LOG_SCALAR(channel, time, value) ((void)0)
    #define LOG_VEC3(channel, time, x, y, z) ((void)0)
#endif




using LogData = std::variant<
    std::vector<float>,
    std::vector<double>,
    std::vector<std::array<float, 3>>,
    std::vector<std::array<double, 3>>
>;

struct LogChannel {
    std::vector<double> timestamps;
    LogData data;
    std::string name;
    
    LogChannel(const std::string& channelName) : name(channelName) {}
};

class dataLogger {
private:
std::map<std::string, std::unique_ptr<LogChannel>> channels;

LogChannel* getOrCreateChannel(const std::string& name) {
    auto it = channels.find(name);
    if (it == channels.end()) {
        auto channel = std::make_unique<LogChannel>(name);
        auto* ptr = channel.get();
        channels[name] = std::move(channel);
        return ptr;
    }
    return it->second.get();
}

public:

dataLogger() = default;
    
void logData(const std::string& channel, double timestamp, float value) {
    auto* ch = getOrCreateChannel(channel);
    ch->timestamps.push_back(timestamp);
    
    if (std::holds_alternative<std::vector<float>>(ch->data)) {
        std::get<std::vector<float>>(ch->data).push_back(value);
    } else {
        ch->data = std::vector<float>{value};
    }
}

void logData(const std::string& channel, double timestamp, double value) {
    auto* ch = getOrCreateChannel(channel);
    ch->timestamps.push_back(timestamp);
    
    if (std::holds_alternative<std::vector<double>>(ch->data)) {
        std::get<std::vector<double>>(ch->data).push_back(value);
    } else {
        ch->data = std::vector<double>{value};
    }
}

void logData(const std::string& channel, double timestamp, const std::array<float, 3>& vec3) {
    auto* ch = getOrCreateChannel(channel);
    ch->timestamps.push_back(timestamp);
    
    if (std::holds_alternative<std::vector<std::array<float, 3>>>(ch->data)) {
        std::get<std::vector<std::array<float, 3>>>(ch->data).push_back(vec3);
    } else {
        ch->data = std::vector<std::array<float, 3>>{vec3};
    }
}

void logData(const std::string& channel, double timestamp, const std::array<double, 3>& vec3) {
    auto* ch = getOrCreateChannel(channel);
    ch->timestamps.push_back(timestamp);
    
    if (std::holds_alternative<std::vector<std::array<double, 3>>>(ch->data)) {
        std::get<std::vector<std::array<double, 3>>>(ch->data).push_back(vec3);
    } else {
        ch->data = std::vector<std::array<double, 3>>{vec3};
    }
}

bool hasChannel(const std::string& channel) const {
    return channels.find(channel) != channels.end();
}
 

bool exportCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }
    
    file << std::fixed << std::setprecision(6);

    for (const auto& pair : channels) {
        const auto& name = pair.first;
        const auto& channel = pair.second;
        
        if (channel->timestamps.empty()) continue;
        
        file << "\n# " << channel->name << "\n";
        
        std::visit([&](auto&& data) {
            using T = std::decay_t<decltype(data)>;
            
            if constexpr (std::is_same_v<T, std::vector<float>> || 
                          std::is_same_v<T, std::vector<double>>) {
                file << "Timestamp,Value\n";
                for (size_t i = 0; i < channel->timestamps.size() && i < data.size(); ++i) {
                    file << channel->timestamps[i] << "," << data[i] << "\n";
                }
            } else if constexpr (std::is_same_v<T, std::vector<std::array<float, 3>>> ||
                                 std::is_same_v<T, std::vector<std::array<double, 3>>>) {
                file << "Timestamp,X,Y,Z\n";
                for (size_t i = 0; i < channel->timestamps.size() && i < data.size(); ++i) {
                    file << channel->timestamps[i] << "," 
                         << data[i][0] << "," 
                         << data[i][1] << "," 
                         << data[i][2] << "\n";
                }
            }
        }, channel->data);
    }
    
    file.close();
    return true;
}

void printSummary() const {
    std::cout << "DataLogger Summary:\n";
    std::cout << "==================\n";
    std::cout << "Total channels: " << channels.size() << "\n\n";
    
    for (const auto& pair : channels) {  
        const auto& name = pair.first;
        const auto& channel = pair.second;
        
        std::cout << channel->name << ": " << channel->timestamps.size() 
                  << " entries\n";
        if (!channel->timestamps.empty()) {
            std::cout << "  Time range: [" << channel->timestamps.front() 
                      << ", " << channel->timestamps.back() << "]\n";
        }
    }
}
};

extern std::unique_ptr<dataLogger> logger;

}
}
#endif 