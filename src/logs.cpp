#include "../include/vehicle.h"
#include "../include/logs.h"
#include "../include/vectorMath.h"
#include "../include/constants.h"
#include <fstream>
#include <string>

std::ofstream outputFile;

void initializeCSV() {
    outputFile.open("data.csv");
    if (!outputFile.is_open()) {
        throw std::runtime_error("Unable to open file: " );
    }
    std::string headers = "DeltaTime,Xposition,Yposition,Zposition,XVehicleState,YVehicleState,ZVehicleState,Velocity,Acceleration";
    appendRowToCSV(headers);
}

void appendRowToCSV(const std::string& row) {
    if (!outputFile.is_open()) {
        throw std::runtime_error("CSV file is not open. Call initializeCSV first.");
    }
    outputFile << row << "\n";
}

void closeCSV() {
    if (outputFile.is_open()) {
        outputFile.close();
    }
}


void logRocketPosition(Vehicle &rocket) {
    rocket.vehicleState = normalizeVector(rocket.vehicleState);
    std::string row = std::to_string(rocket.iterations * constants::timeStep) + "," + 
                    std::to_string(rocket.Xposition) + "," + 
                    std::to_string(rocket.Yposition) + "," +
                    std::to_string(rocket.Zposition) + "," + 
                    std::to_string(rocket.vehicleState[0])+ "," +
                    std::to_string(rocket.vehicleState[1])+ "," +
                    std::to_string(rocket.vehicleState[2])+ "," +
                    std::to_string(rocket.getVelocity())+ "," +
                    std::to_string(rocket.gForce)+ "," +
                    std::to_string(rocket.engineState[0])+ "," +
                    std::to_string(rocket.engineState[1])+ "," +
                    std::to_string(rocket.engineState[2])+ "," ;
                    //std::to_string(rocket.logEngineVector[0])+ "," +
                    //std::to_string(rocket.logEngineVector[1])+ "," +
                    //std::to_string(rocket.logEngineVector[2])+ ",";



    appendRowToCSV(row);
}