#ifndef CSV_HANDLER_H
#define CSV_HANDLER_H

#include <string>


void initializeCSV();

void appendRowToCSV(const std::string& row);

void closeCSV();

void logRocketPosition(Vehicle &rocket , int iterations);

#endif 