#include <math.h>
#include <array>
#include "../include/vectorMath.h"



float vectorMag(std::array<float,3> &vector){
    return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
}


float vectorDotProduct(std::array<float,3> &vector1, std::array<float,3> &vector2) {
    float dotProduct = 0.0;

    //v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    for (int i = 0; i < 3; i++) {
        dotProduct += vector1[i] * vector2[i];
    }

    return dotProduct;
}


void vectorCrossProduct(std::array<float,3> &vector1, std::array<float,3> &vector2, std::array<float,3> &result) {

    result[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1]; 
    result[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2]; 
    result[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0]; 
}




float vectorAngleBetween(std::array<float,3> &vector1, std::array<float,3> &vector2) {
    
    float mag1 = vectorMag(vector1);
    float mag2 = vectorMag(vector2);
    float dot = vectorDotProduct(vector1, vector2);
    
    if(mag1 == 0 || mag2 == 0) return 0;
    return acos(dot / (mag1 * mag2)); // Result is in radians
}



std::array<float,3> normalizeVector(std::array<float,3> &vector1){
    std::array<float,3> normalVector;
    float mag = vectorMag(vector1);
    if(mag == 0) return normalVector = {0,0,0};
    for(int i = 0 ; i< 3 ; i++) {
        normalVector[i] = vector1[i]/mag;
    }
    return normalVector;
}








