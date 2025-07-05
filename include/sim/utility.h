#pragma once
#include <iostream>
#include <iomanip>

// percentage: float between 0.0 and 1.0
inline void progressBar(float percentage) {
    static const int barWidth = 50;
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
