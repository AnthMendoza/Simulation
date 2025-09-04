#include "../../include/dynamics/vehicle.h"
#include "../../include/sim/logs.h"
#include "../../include/core/vectorMath.h"

#include "../../include/subsystems/sensors.h"
#include "../../include/dynamics/rocket.h"
#include <fstream>
#include <string>
#include <vector>

#include <stdexcept>
#include <cmath>
#include <fstream>

namespace SimCore{
loggedData::loggedData(int preset){

    timeStepVect->reserve(preset);
    Xposition->reserve(preset);
    Yposition->reserve(preset);
    Zposition->reserve(preset);
    vehicleState0->reserve(preset);
    vehicleState1->reserve(preset);
    vehicleState2->reserve(preset);
    absVelocity->reserve(preset);
    gForce->reserve(preset);
    gimbalXAngle->reserve(preset);
    gimbalYAngle->reserve(preset);
    mass->reserve(preset);
    fuel->reserve(preset);
    LOX->reserve(preset);
    engineVector0->reserve(preset);
    engineVector1->reserve(preset);
    engineVector2->reserve(preset);
    enginePower->reserve(preset);
    
}


void loggedData::logRocketPosition(Rocket &rocket ) {

    timeStepVect->push_back(rocket.getIterations() * rocket.getTimeStep());
    auto pos = rocket.getPositionVector();
    Xposition->push_back(pos[0]);
    Yposition->push_back(pos[1]);
    Zposition->push_back(pos[2]);
    auto state = rocket.getState();
    vehicleState0->push_back(state[0]);
    vehicleState1->push_back(state[1]);
    vehicleState2->push_back(state[2]);

    absVelocity->push_back(rocket.getVelocity());
    gForce->push_back(rocket.getGForce());

    gimbalXAngle->push_back(rocket.gimbalX);
    gimbalYAngle->push_back(rocket.gimbalY);
        
    mass->push_back(rocket.getMass());
    fuel->push_back(rocket.fuel);
    LOX->push_back(rocket.LOX);

    engineVector0->push_back(rocket.engineState[0]);
    engineVector1->push_back(rocket.engineState[1]);
    engineVector2->push_back(rocket.engineState[2]);

    enginePower->push_back(rocket.enginePower); //number from 0 to 1
    stateEstimationVelocityX->push_back(rocket.getEstimatedVelocity()[0]);
    stateEstimationVelocityY->push_back(rocket.getEstimatedVelocity()[1]);
    stateEstimationVelocityZ->push_back(rocket.getEstimatedVelocity()[2]);
    stateEstimationPositionX->push_back(rocket.getEstimatedPosition()[0]);
    stateEstimationPositionY->push_back(rocket.getEstimatedPosition()[1]);
    stateEstimationPositionZ->push_back(rocket.getEstimatedPosition()[2]);

    specificEnergy->push_back(rocket.getSpecificEnergy());

    absEstimatedVelocity->push_back(rocket.getAbsEstimatedVelocity());

}
    
void loggedData::lowPrecisionData(std::vector<float> &data , std::vector<float> &returnData ,  int desiredResolution){

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


std::vector<std::shared_ptr<std::vector<float>>> loggedData::all(){
    return {timeStepVect,
            Xposition,
            Yposition,
            Zposition,
            vehicleState0,
            vehicleState1,
            vehicleState2,
            absVelocity,
            gForce,
            gimbalXAngle,
            gimbalYAngle,
            fuel,
            mass,
            LOX,
            engineVector0,
            engineVector1,
            engineVector2,
            enginePower,
            stateEstimationVelocityX,
            stateEstimationVelocityY,
            stateEstimationVelocityZ,
            stateEstimationPositionX,
            stateEstimationPositionY,
            stateEstimationPositionZ,
            specificEnergy,
            absEstimatedVelocity
            
        };
}

void loggedData::writeCSV( const std::string& filename,const std::vector<std::shared_ptr<std::vector<float>>>& data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        //handle error
        return;
    }
    if(data.empty()) return;
    if(!header.empty()) file << header << "\n";
    int numRows = (*(data[0])).size();
    for(int i = 0; i < numRows; ++i) {
        for(int j = 0 ; j < data.size() ; j++){
            if(j<data.size() - 1 )file << (*data[j])[i]<<",";
            else file << (*data[j])[i];
        }
        file << "\n";
    }

    file.close();
}
void writeCSV( const std::string& filename,const std::vector<std::shared_ptr<std::vector<float>>>& data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        //handle error
        return;
    }
    if(data.empty()) return;
    int numRows = (*(data[0])).size();
    for(int i = 0; i < numRows; ++i) {
        for(int j = 0 ; j < data.size() ; j++){
            if(j<data.size() - 1 )file << (*data[j])[i]<<",";
            else file << (*data[j])[i];
        }
        file << "\n";
    }

    file.close();
}





/*
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>




void dataToRam(char* unique_id){

    //reduce size
    std::vector<float> timeStepVectReduced;
    std::vector<float> gimbalXAngleReduced;
    std::vector<float> gimbalYAngleReduced;
    std::vector<float> massReduced;
    std::vector<float> fuelReduced;
    std::vector<float> LOXReduced;

    int lowRes = 500;

    lowPrecisionData(timeStepVect , timeStepVectReduced , lowRes);
    lowPrecisionData(gimbalXAngle , gimbalXAngleReduced , lowRes);
    lowPrecisionData(gimbalYAngle , gimbalYAngleReduced , lowRes);
    lowPrecisionData(mass , massReduced , lowRes);
    lowPrecisionData(fuel , fuelReduced , lowRes);
    lowPrecisionData(LOX , LOXReduced , lowRes);

    const char *shm_name = unique_id;
    const size_t SIZE = 1024 * 10000; // 10 KB of shared memory
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);

    if (shm_fd == -1) {
        std::cerr << "Failed to open shared memory." << std::endl;
    }
    std::cout<< "Xposition size : "<<Xposition.size()<< std::endl;
    std::cout<<"Yposition size : "<< Yposition.size()<< std::endl;
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

    int_ptr[11] = massReduced.size();                                                                                                                                                                                
    int_ptr[12] = fuelReduced.size();
    int_ptr[13] = LOXReduced.size();
    int_ptr[14] = timeStepVectReduced.size();
    int_ptr[15] = engineVector0.size();
    int_ptr[16] = engineVector1.size();
    int_ptr[17] = engineVector2.size();
    int_ptr[18] = enginePower.size();



    // Write the contents of the arrays
    int count = 19; // number of arrays being logged*****
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
    count += gimbalYAngleReduced.size();
    std::memcpy(&int_ptr[count], massReduced.data(), massReduced.size() * sizeof(float));
    count += massReduced.size();
    std::memcpy(&int_ptr[count], fuelReduced.data(), fuelReduced.size() * sizeof(float));
    count += fuelReduced.size();
    std::memcpy(&int_ptr[count], LOXReduced.data(), LOXReduced.size() * sizeof(float));
    count += LOXReduced.size();
    std::memcpy(&int_ptr[count], timeStepVectReduced.data(), timeStepVectReduced.size() * sizeof(float));
    count += timeStepVectReduced.size();
    std::memcpy(&int_ptr[count], engineVector0.data(), engineVector0.size() * sizeof(float));
    count += engineVector0.size();
    std::memcpy(&int_ptr[count], engineVector1.data(), engineVector1.size() * sizeof(float));
    count += engineVector1.size();
    std::memcpy(&int_ptr[count], engineVector2.data(), engineVector2.size() * sizeof(float));
    count += engineVector2.size();
    std::memcpy(&int_ptr[count], enginePower.data(), enginePower.size() * sizeof(float));

    munmap(ptr, SIZE);
    close(shm_fd);

    std::cout << "Data written to shared memory." << std::endl;

}
*/

}