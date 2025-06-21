#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <array>
#include <stdexcept>
namespace SimCore{

template <typename T>
T vectorMag(const std::array<T,3> &vector){
    return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
}


template <typename T>
T vectorDotProduct(const std::array<T,3> &vector1,const std::array<T,3> &vector2) {
    T dotProduct = 0.0;

    //v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    for (int i = 0; i < 3; i++) {
        dotProduct += vector1[i] * vector2[i];
    }

    return dotProduct;
}


template <typename T>
void vectorCrossProduct(const std::array<T,3> &vector1,const std::array<T,3> &vector2, std::array<T,3> &result) {

    result[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1]; 
    result[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2]; 
    result[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0]; 
}


template <typename T>
T vectorAngleBetween(const std::array<T,3> &vector1,const std::array<T,3> &vector2) {
    
    T mag1 = vectorMag(vector1);
    T mag2 = vectorMag(vector2);
    T dot = vectorDotProduct(vector1, vector2);

  
    if(mag1 == 0 || mag2 == 0) return 0;
    T dotOverMags = dot / (mag1 * mag2);
    if(dotOverMags < -1) return 3.1415;
    if(dotOverMags > 1) return 0;
    return acos(dotOverMags); // Result is in radians
}


template <typename T>
std::array<T,3> normalizeVector(const std::array<T,3> vector1){
    std::array<T,3> normalVector;
    T mag = vectorMag(vector1);
    if(mag == 0) return normalVector = {0,0,0};
    for(int i = 0 ; i< 3 ; i++) {
        normalVector[i] = vector1[i]/mag;
    }
    return normalVector;
}


template <typename T>
T twodAngleDiffrence(const std::array<T , 2> &vector1 ,const std::array<T , 2> &vector2){

    T vectDot = vector1[0] * vector2[0] + vector1[1] * vector2[1];

    T mag1 = sqrtf(vector1[0] * vector1[0] + vector1[1] * vector1[1]);
    T mag2 = sqrtf(vector2[0] * vector2[0] + vector2[1] * vector2[1]);

    if(mag1 == 0 || mag2 == 0) return 0;

    T insideCos = (vectDot)/(mag1 * mag2);

    if(insideCos > 1){
        insideCos = 1;
    }else if(insideCos < -1){
        insideCos = -1;
    }

    T angle = acosf(insideCos);

    T cross = (vector1[0] * vector2[1] - vector1[1] * vector2[0]);
    
    if(cross < 0){
        return -angle;
    }
    return angle;

}


template <typename T>
std::array<T , 3> addVectors(const std::array<T , 3> &vec1 ,const std::array<T , 3> &vec2){
    std::array<T , 3> addedVector= {vec1[0]+vec2[0],vec1[1]+vec2[1],vec1[2]+vec2[2]};
    return addedVector;
}


template <typename T>
bool directionality(std::array<T , 3> &refranceVector ,std::array<T , 3> &vector1){

    std::array<T , 3> normalRefranceVector = normalizeVector(refranceVector);
    std::array<T , 3> normalVector1 = normalizeVector(vector1);

    T sqrtTwo = 1.414213562;
    std::array<T,3> sumOfVect = addVectors(normalRefranceVector,normalVector1);
    T chordLength = vectorMag(sumOfVect);

    if( chordLength > sqrtTwo) return false;
    return true;
}


template <typename T>
void setInBounds(T &value , const T &lower , const T &upper){
    
    if(value > upper) value = upper;
    if(value < lower) value = lower;

}

template <typename T>
inline bool isZeroVector(T &vector){
    if(vector[0] == 0 && vector[1] == 0 && vector[2] == 0) return true;
    return false;
}

}

#endif