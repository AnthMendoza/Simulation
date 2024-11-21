#include "../include/vehicle.h"
#include "../include/logs.h"
#include "../include/vectorMath.h"
#include "../include/constants.h"
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>

std::ofstream outputFile;
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



void initializeCSV() {
    outputFile.open("data.csv");
    if (!outputFile.is_open()) {
        throw std::runtime_error("Unable to open file: " );
    }
    std::string headers = "Time,Xposition,Yposition,Zposition,XVehicleState,YVehicleState,ZVehicleState,Velocity,Acceleration,XengineState,YengineState,ZengineState,gimalXerror,error,xtwodAngle,ytwodangle";
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

void initializeVectors(int preset){
    #ifdef __linux__
        timeStepVect.reserve(preset);
        Xposition.reserve(preset);
        Yposition.reserve(preset);
        Zposition.reserve(preset);
        vehicleState0.reserve(preset);
        vehicleState1.reserve(preset);
        vehicleState2.reserve(preset);
        absVelocity.reserve(preset);
        gForce.reserve(preset);
        gimbalXAngle.resever(preset);
        gimbalYAngle.resever(preset);
    #endif
}




void logRocketPosition(Vehicle &rocket) {
    if(constants::isLinux == true){
        timeStepVect.push_back(rocket.iterations * constants::timeStep);

        Xposition.push_back(rocket.Xposition);
        Yposition.push_back(rocket.Yposition);
        Zposition.push_back(rocket.Zposition);

        vehicleState0.push_back(rocket.vehicleState[0]);
        vehicleState1.push_back(rocket.vehicleState[1]);
        vehicleState2.push_back(rocket.vehicleState[2]);

        absVelocity.push_back(rocket.getVelocity());
        gForce.push_back(rocket.gForce);

        gimbalXAngle.push_back(rocket.gimbalX);
        gimbalYAngle.push_back(rocket.gimbalY);



    }else{

        rocket.vehicleState = normalizeVector(rocket.vehicleState);
        std::string row = std::to_string(rocket.iterations * constants::timeStep*1.4) + "," + 
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
                        std::to_string(rocket.engineState[2])+ "," +

                        std::to_string(rocket.gimbalXError)+ "," +

                        std::to_string(rocket.error)+ "," +
                        std::to_string(rocket.gimbalYError)+ "," +
                        std::to_string(rocket.twoDAngle[1])+ "," ;

        appendRowToCSV(row);

    }
}

void lowPrecisionData(std::vector<float> &data , std::vector<float> &returnData ,  int desiredResolution){

    if(desiredResolution <= 0) throw std::invalid_argument ("desiredResolution cannot be zero");

    float ratio  = data.size()/desiredResolution;
    int roundedRatio = std::floor(ratio);

    if(roundedRatio < 2){
        returnData = data;
        return;
    }

    for(int i = 0 ; i < data.size() ; i++){
        if( i % roundedRatio == 0 ){
            returnData.push_back(data[i]);
        }
    }

}


#ifdef __linux__

#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>




void dataToRam(char* unique_id){

    //reduce size
    std::vector<float> gimbalXAngleReduced;
    std::vector<float> gimbalYAngleReduced;
    lowPrecisionData(gimbalXAngle , gimbalXAngleReduced , 500);
    lowPrecisionData(gimbalYAngle , gimbalYAngleReduced , 500);

    const char *shm_name = unique_id;
    const size_t SIZE = 1024 * 6000; // 10 KB of shared memory
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);

    if (shm_fd == -1) {
        std::cerr << "Failed to open shared memory." << std::endl;
    }
    std::cout<< "Xposition size : "<<Xposition.size()<< std::endl;
    std::cout<<"Yposition size : "<< Yposition.size();
    // Set the size of the shared memory object
    ftruncate(shm_fd, SIZE);

    // Map shared memory object
    void *ptr = mmap(0, SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        std::cerr << "Failed to map shared memory." << std::endl;
    }


    // Write the lengths of the arrays
    int *int_ptr = static_cast<int*>(ptr);
    int_ptr[0] = timeStepVect.size();
    int_ptr[1] = Xposition.size();
    int_ptr[2] = Yposition.size();
    int_ptr[3] = Zposition.size();

    int_ptr[4] = vehicleState0.size();
    int_ptr[5] = vehicleState1.size();  
    int_ptr[6] = vehicleState2.size();  
    int_ptr[7] = absVelocity.size();  
    int_ptr[8] = gForce.size();
    int_ptr[9] =  gimbalXAngleReduced.size(); 
    int_ptr[10] = gimbalYAngleReduced.size();                                                                                                                                                                                            


    // Write the contents of the arrays
    int count = 11;
    std::memcpy(&int_ptr[count], timeStepVect.data(), timeStepVect.size() * sizeof(float));
    count += timeStepVect.size();
    std::memcpy(&int_ptr[count], Xposition.data(), Xposition.size() * sizeof(float));
    count += Xposition.size();
    std::memcpy(&int_ptr[count], Yposition.data(), Yposition.size() * sizeof(float));
    count += Yposition.size();
    std::memcpy(&int_ptr[count], Zposition.data(), Zposition.size() * sizeof(float));
    count += Zposition.size();
    std::memcpy(&int_ptr[count], vehicleState0.data(), vehicleState0.size() * sizeof(float));
    count += vehicleState0.size();
    std::memcpy(&int_ptr[count], vehicleState1.data(), vehicleState1.size() * sizeof(float));
    count += vehicleState1.size();
    std::memcpy(&int_ptr[count], vehicleState2.data(), vehicleState2.size() * sizeof(float));
    count += vehicleState2.size();
    std::memcpy(&int_ptr[count], absVelocity.data(), absVelocity.size() * sizeof(float));
    count += absVelocity.size();
    std::memcpy(&int_ptr[count], gForce.data(), gForce.size() * sizeof(float));

    count += gForce.size();
    std::memcpy(&int_ptr[count], gimbalXAngleReduced.data(), gimbalXAngleReduced.size() * sizeof(float));
    count += gimbalXAngleReduced.size();
    std::memcpy(&int_ptr[count], gimbalYAngleReduced.data(), gimbalYAngleReduced.size() * sizeof(float));

    std::cout << "Data written to shared memory." << std::endl;

    // Clean up
    munmap(ptr, SIZE);
    close(shm_fd);

}


#endif