#include "../include/getRotation.h"
#include "../include/vectorMath.h"
#include "../include/Eigen/Eigen"
#include "../include/Eigen/Dense"
#include <iostream>
#include <array>


using namespace std;

int main(){
    array<float,3> zrefrance = {0,-1,3};
    array<float,3> vehicleState = {0,0,1};

    zrefrance = normalizeVector(zrefrance);


    Eigen::Vector3d v1( zrefrance[0],
                        zrefrance[1],
                        zrefrance[2]);


    Eigen::Vector3d v2( vehicleState[0],
                        vehicleState[1],
                        vehicleState[2]);
    
    Eigen::Vector3d rotation = getRotationAngles(v1, v2);

    cout<<rotation(0) << " " << rotation(1) << " " << rotation(2);
}