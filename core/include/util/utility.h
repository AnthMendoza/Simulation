#pragma once
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <fstream>
#include <spdlog/spdlog.h>

namespace utility{

// percentage: float between 0.0 and 1.0
inline void progressBar(float percentage) {
    static const int barWidth = 100;
    int filled = static_cast<int>(percentage * barWidth);

    //std::cout << "\x1b[1A" << "\x1b[2K";
    std::cout << "[";
    for (int i = 0; i < barWidth; ++i) {
        if (i < filled) std::cout << "=";
        else if (i == filled) std::cout << ">";
        else std::cout << " ";
    }

    std::cout << "] " << std::fixed << std::setprecision(1) << percentage * 100 << "%\n";
    std::cout << std::flush;
}

template<typename T>
inline void print(const T& value, const char* name) {
    std::cout << name << ": ["<<value;
    std::cout << "]\n";
}

template<typename T , std::size_t N>
inline void print(const std::shared_ptr<spdlog::logger>& logger,const std::array<T, N>& arr, const char* name){
    std::ostringstream oss;
    oss << name << ": [";

    for (std::size_t i = 0; i < N; ++i) {
        oss << arr[i];
        if (i < N - 1) oss << ", ";
    }

    oss << "]";
    SPDLOG_LOGGER_INFO(logger, "{}", oss.str());
}


template<typename T>
inline void print(const std::shared_ptr<spdlog::logger>& logger,const T& value, const char* name) {
    SPDLOG_LOGGER_INFO(logger, "{}: {}", name, value);
}


template<typename T, std::size_t N>
inline void print(const std::array<T, N>& arr, const char* name, int precision = 5) {
    std::streamsize oldPrecision = std::cout.precision();

    std::cout << std::fixed << std::setprecision(precision);
    std::cout << name << ": [";
    for (size_t i = 0; i < N; ++i) {
        std::cout << arr[i];
        if (i != N - 1)
            std::cout << ", ";
    }
    std::cout << "]\n";
    std::cout.precision(oldPrecision);
}


template<typename T>
inline void print(const std::vector<T>& vec, const char* name) {
    std::cout << name << ": [";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i != vec.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]\n";
}

inline std::string readFileAsString(const std::string& filePath) {
    std::ifstream inFile(filePath);
    if(!inFile.is_open()){
        std::cerr<< "Path to File not found : "<< filePath << "\n";
        std::string nullString = "";
        return nullString;
    }
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    return buffer.str();
}


inline void printDynamicDisplay(const std::string& output) {
    int linesToClear = std::count(output.begin(), output.end(), '\n');

    for (int i = 0; i < linesToClear; ++i){
        std::cout << "\x1b[1A" << "\x1b[2K";
    }
    std::cout << output;
}


enum class direction {
    positive = 1,
    negative = -1
};


inline bool validateDirection(float val, direction dir) {
    if (dir == direction::positive && val > 0)
        return true;
    if (dir == direction::negative && val < 0)
        return true;
    return false;
}

}