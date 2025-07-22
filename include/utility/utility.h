#pragma once
#include <iostream>
#include <iomanip>

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


template<typename T, std::size_t N>
inline void printArray(const std::array<T, N>& arr, const char* name) {
    std::cout << name << ": [";
    for (size_t i = 0; i < N; ++i) {
        std::cout << arr[i];
        if (i != N - 1)
            std::cout << ", ";
    }
    std::cout << "]\n";
}


template<typename T>
inline void printVector(const std::vector<T>& vec, const char* name) {
    std::cout << name << ": [";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i != vec.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]\n";
}


template<typename T>
inline void print(const T& value, const char* name) {
    std::cout << name << ": ["<<value;
    std::cout << "]\n";
}