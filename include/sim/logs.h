#ifndef CSV_HANDLER_H
#define CSV_HANDLER_H

#include <string>
#include <vector>
#include "../../include/subsystems/sensors.h"
#include <fstream>

#include <memory>
#include "../dynamics/vehicle.h"
#pragma once
namespace SimCore{
using vecFloat = std::shared_ptr<std::vector<float>>;

class Rocket;
class loggedData{
    public:
    vecFloat timeStepVect = std::make_shared<std::vector<float>>();
    vecFloat Xposition = std::make_shared<std::vector<float>>();
    vecFloat Yposition = std::make_shared<std::vector<float>>();
    vecFloat Zposition = std::make_shared<std::vector<float>>();
    vecFloat vehicleState0 = std::make_shared<std::vector<float>>();
    vecFloat vehicleState1 = std::make_shared<std::vector<float>>();
    vecFloat vehicleState2 = std::make_shared<std::vector<float>>();
    vecFloat absVelocity = std::make_shared<std::vector<float>>();
    vecFloat gForce = std::make_shared<std::vector<float>>();
    vecFloat gimbalXAngle = std::make_shared<std::vector<float>>();
    vecFloat gimbalYAngle = std::make_shared<std::vector<float>>();
    vecFloat fuel = std::make_shared<std::vector<float>>();
    vecFloat mass = std::make_shared<std::vector<float>>();
    vecFloat LOX = std::make_shared<std::vector<float>>();
    vecFloat engineVector0 = std::make_shared<std::vector<float>>();
    vecFloat engineVector1 = std::make_shared<std::vector<float>>();
    vecFloat engineVector2 = std::make_shared<std::vector<float>>();
    vecFloat enginePower = std::make_shared<std::vector<float>>();
    vecFloat stateEstimationVelocityX = std::make_shared<std::vector<float>>();
    vecFloat stateEstimationVelocityY = std::make_shared<std::vector<float>>();
    vecFloat stateEstimationVelocityZ = std::make_shared<std::vector<float>>();
    vecFloat stateEstimationPositionX = std::make_shared<std::vector<float>>();
    vecFloat stateEstimationPositionY = std::make_shared<std::vector<float>>();
    vecFloat stateEstimationPositionZ = std::make_shared<std::vector<float>>();
    vecFloat specificEnergy = std::make_shared<std::vector<float>>();
    vecFloat absEstimatedVelocity = std::make_shared<std::vector<float>>();

    std::string header = "timeStepVect,Xposition,Yposition,Zposition,vehicleState0,vehicleState1,vehicleState2,absVelocity,gForce,gimbalXAngle,gimbalYAngle,fuel,mass,LOX,engineVector0,engineVector1,engineVector2,enginePower,stateEstimationVelocityX,stateEstimationVelocityY,stateEstimationVelocityZ,stateEstimationPositionX,stateEstimationPositionY,stateEstimationPositionZ,specificEnergy,AbsEstimatedVelocity";

    loggedData(int preset = 10000);
    
    std::vector<std::shared_ptr<std::vector<float>>> all();
    
    void logRocketPosition(Rocket &rocket);

    void logRocketPosition();

    void lowPrecisionData(std::vector<float> &data , std::vector<float> &returnData ,  int desiredResolution);

    void writeCSV(const std::string& filename,const std::vector<std::shared_ptr<std::vector<float>>>& data);

};

void writeCSV(const std::string& filename,const std::vector<std::shared_ptr<std::vector<float>>>& data);


/*
void initializeCSV();

void initializeVectors(int preset);

void appendRowToCSV(const std::string& row);

void closeCSV();
*/

}

#endif 