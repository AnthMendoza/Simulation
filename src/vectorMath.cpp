#include <math.h>
#include <array>
#include <iostream>
#include <stdexcept>
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
    float dotOverMags = dot / (mag1 * mag2);
    if(dotOverMags < -1) return 3.1415;
    if(dotOverMags > 1) return 0;
    return acos(dotOverMags); // Result is in radians
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



float twodAngleDiffrence(std::array<float , 2> &vector1 , std::array<float , 2> &vector2){

    float vectDot = vector1[0] * vector2[0] + vector1[1] * vector2[1];

    float mag1 = sqrtf(vector1[0] * vector1[0] + vector1[1] * vector1[1]);
    float mag2 = sqrtf(vector2[0] * vector2[0] + vector2[1] * vector2[1]);

    if(mag1 == 0 || mag2 == 0) return 0;

    float insideCos = (vectDot)/(mag1 * mag2);

    if(insideCos > 1){
        insideCos = 1;
    }else if(insideCos < -1){
        insideCos = -1;
    }

    float angle = acosf(insideCos);

    float cross = (vector1[0] * vector2[1] - vector1[1] * vector2[0]);
    //std::cout<< angle <<" " << cross << std::endl;
    if(cross < 0){
        return -angle;
    }
    return angle;

}


std::array<float , 3> addVectors(std::array<float , 3> &vec1 , std::array<float , 3> &vec2){
    std::array<float , 3> addedVector= {vec1[0]+vec2[0],vec1[1]+vec2[1],vec1[2]+vec2[2]};
    return addedVector;
}



bool directionality(std::array<float , 3> &refranceVector ,std::array<float , 3> &vector1){

    std::array<float , 3> normalRefranceVector = normalizeVector(refranceVector);
    std::array<float , 3> normalVector1 = normalizeVector(vector1);

    float sqrtTwo = 1.414213562;
    std::array<float,3> sumOfVect = addVectors(normalRefranceVector,normalVector1);
    float chordLength = vectorMag(sumOfVect);

    if( chordLength > sqrtTwo) return false;
    return true;
}






