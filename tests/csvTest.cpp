#include <iostream>
#include "../include/logs.h"

std::ofstream outputFile;

void main() {
    initCSV();
    if (outputFile.is_open()) {
        
        outputFile << 5 << "," << 10 << "," << 5 << "\n";
    } else {
        std::cerr << "File not open. Call initCSV() before logging data!" << std::endl;
    }
    closeCSV();
}