#ifndef CSV_HANDLER_H
#define CSV_HANDLER_H

#include <string>
#include <vector>
extern std::vector<float> timeStampVect;
extern std::vector<float> Xposition;
extern std::vector<float> Yposition;
extern std::vector<float> Zposition;
extern std::vector<float> vehicleState0;
extern std::vector<float> vehicleState1;
extern std::vector<float> vehicleState2;
extern std::vector<float> absVelocity;
extern std::vector<float> gForce;



void initializeCSV();

void initializeVectors(int preset);

void appendRowToCSV(const std::string& row);

void closeCSV();

void logRocketPosition(Vehicle &rocket);
#ifdef __linux__

void dataToRam(char* unique_id);

#endif

#endif 