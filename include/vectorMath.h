#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H



float vectorMag(std::array<float,3> &vector);

float vectorDotProduct(std::array<float,3> &vector1, std::array<float,3> &vector2);

void vectorCrossProduct(std::array<float,3> &vector1, std::array<float,3> vector2, std::array<float,3> result);

float vectorAngleBetween(std::array<float,3> &vector1, std::array<float,3> &vector2);

std::array<float,3> normalizeVector(std::array<float,3> &vector1);

float twodAngleDiffrence(std::array<float , 2> &vector1 , std::array<float , 2> &vector2);

std::array<float , 3> addVectors(std::array<float , 3> &vec1 , std::array<float , 3> &vec2);

bool directionality(std::array<float , 3> &refranceVector ,std::array<float , 3> &vector1);



#endif