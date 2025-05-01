#ifndef CSV_HANDLER_H
#define CSV_HANDLER_H

#include <string>
#include <vector>
#include "../include/sensors.h"
#include <fstream>
#include <iostream>

#pragma once
class loggedData{
    public:
    std::vector<float> timeStepVect;
    std::vector<float> Xposition;
    std::vector<float> Yposition;
    std::vector<float> Zposition;
    std::vector<float> vehicleState0;
    std::vector<float> vehicleState1;
    std::vector<float> vehicleState2;
    std::vector<float> absVelocity;
    std::vector<float> gForce;
    std::vector<float> gimbalXAngle;
    std::vector<float> gimbalYAngle;
    std::vector<float> fuel;
    std::vector<float> mass;
    std::vector<float> LOX;
    std::vector<float> engineVector0;
    std::vector<float> engineVector1;
    std::vector<float> engineVector2;
    std::vector<float> enginePower;

    std::vector<float> stateEstimationVelocityX;
    std::vector<float> stateEstimationVelocityY;
    std::vector<float> stateEstimationVelocityZ;
    std::vector<float> stateEstimationPositionX;
    std::vector<float> stateEstimationPositionY;
    std::vector<float> stateEstimationPositionZ;

    std::string header = "timeStepVect,Xposition,Yposition,Zposition,vehicleState0,vehicleState1,vehicleState2,absVelocity,gForce,gimbalXAngle,gimbalYAngle,fuel,mass,LOX,engineVector0,engineVector1,engineVector2,enginePower,stateEstimationVelocityX,stateEstimationVelocityY,stateEstimationVelocityZ,stateEstimationPositionX,stateEstimationPositionY,stateEstimationPositionZ";

    loggedData(int preset);

    loggedData();
    
    std::vector<std::vector<float>*> all();
    
    void logRocketPosition(Vehicle &rocket);

    void logRocketPosition();

    void lowPrecisionData(std::vector<float> &data , std::vector<float> &returnData ,  int desiredResolution);

    void writeCSV(const std::string& filename,const std::vector<std::vector<float>*>& data);

};






/*
void initializeCSV();

void initializeVectors(int preset);

void appendRowToCSV(const std::string& row);

void closeCSV();
*/



#endif 