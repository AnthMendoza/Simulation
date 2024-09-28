#include <iostream>
#include <iomanip>
#include <array>
#include "../include/vectorMath.h"

void testVectorMag() {
    std::array<float,3> vec = {3.0, 4.0, 0.0}; // Magnitude should be 5.0
    float result = vectorMag(vec);
    std::cout << "Test vectorMag: Expected 5.0, Got " << result << std::endl;

    std::array<float,3> vec = {-3.0, -4.0, -5.0}; // Magnitude should be 5.0
    float result = vectorMag(vec);
    std::cout << "Test vectorMag: Expected 7.071, Got " << result << std::endl;
}

void testVectorDotProduct() {
    std::array<float,3> vec1 = {1.0, 0.0, 0.0};
    std::array<float,3> vec2 = {0.0, 1.0, 0.0}; // Dot product should be 0
    float result = vectorDotProduct(vec1, vec2);
    std::cout << "Test vectorDotProduct: Expected 0.0, Got " << result << std::endl;
}

void testVectorCrossProduct() {
    std::array<float,3> vec1 = {1.0, 0.0, 0.0};
    std::array<float,3> vec2 = {0.0, 1.0, 0.0}; // Cross product should be {0, 0, 1}
    std::array<float,3> result = {0.0, 0.0, 0.0};
    vectorCrossProduct(vec1, vec2, result);
    std::cout << "Test vectorCrossProduct: Expected {0.0, 0.0, 1.0}, Got {" << result[0] << ", " << result[1] << ", " << result[2] << "}" << std::endl;
}

void testVectorAngleBetween() {
    std::array<float,3> vec1 = {1.0, 0.0, 0.0};
    std::array<float,3> vec2 = {0.0, 1.0, 0.0}; // Angle should be 90 degrees (π/2 radians)
    float result = vectorAngleBetween(vec1, vec2);
    std::cout << std::setprecision(4) << "Test vectorAngleBetween: Expected 1.5708 radians, Got " << result << " radians" << std::endl;
}



void testNormalize(){
    std::array<float,3> vec1 = {10,2,3};
    std::array<float,3> vec2 = normalizeVector(vec1);
    std::cout<< vec2[0] << " " << vec2[1] << " " << vec2[2] ;
}

int main() {
    std::cout << "Running tests" << std::endl;
    
    testVectorMag();
    testVectorDotProduct();
    testVectorCrossProduct();
    testVectorAngleBetween();
    testNormalize();
    
    std::cout << "All tests completed." << std::endl;
    return 0;
}
