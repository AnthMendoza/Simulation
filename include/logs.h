#ifndef CSV_HANDLER_H
#define CSV_HANDLER_H

#include <string>
#include <vector>


void initializeCSV();

void initializeVectors(int preset);

void appendRowToCSV(const std::string& row);

void closeCSV();

void logRocketPosition(Vehicle &rocket);
#ifdef __linux__

void dataToRam(char* unique_id);

#endif

#endif 