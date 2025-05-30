#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <array>
namespace SimCore{

float vectorMag(const std::array<float,3> &vector);

float vectorDotProduct(const std::array<float,3> &vector1, std::array<float,3> &vector2);

void vectorCrossProduct(const std::array<float,3> &vector1,const std::array<float,3> &vector2, std::array<float,3> &result);

float vectorAngleBetween(const std::array<float,3> &vector1,const std::array<float,3> &vector2);

std::array<float,3> normalizeVector(const std::array<float,3> vector1);

float twodAngleDiffrence(const std::array<float , 2> &vector1 ,const std::array<float , 2> &vector2);

std::array<float , 3> addVectors(const std::array<float , 3> &vec1 ,const std::array<float , 3> &vec2);

bool directionality(std::array<float , 3> &refranceVector ,std::array<float , 3> &vector1);

template <typename T>
void setInBounds(T &value , const T &lower , const T &upper){
    
    if(value > upper) value = upper;
    if(value < lower) value = lower;

}
}

#endif