#include <iostream>
#include <iomanip> // For controlling output precision
#include "../include/vectorMath.h"

void testVectorMag() {
    float vec[3] = {3.0, 4.0, 0.0}; // Magnitude should be 5.0
    float result = vectorMag(vec);
    std::cout << "Test vectorMag: Expected 5.0, Got " << result << std::endl;

    float vec[3] = {-3.0, -4.0, -5.0}; // Magnitude should be 5.0
    float result = vectorMag(vec);
    std::cout << "Test vectorMag: Expected 7.071, Got " << result << std::endl;
}

void testVectorDotProduct() {
    float vec1[3] = {1.0, 0.0, 0.0};
    float vec2[3] = {0.0, 1.0, 0.0}; // Dot product should be 0
    float result = vectorDotProduct(vec1, vec2);
    std::cout << "Test vectorDotProduct: Expected 0.0, Got " << result << std::endl;
}

void testVectorCrossProduct() {
    float vec1[3] = {1.0, 0.0, 0.0};
    float vec2[3] = {0.0, 1.0, 0.0}; // Cross product should be {0, 0, 1}
    float result[3] = {0.0, 0.0, 0.0};
    vectorCrossProduct(vec1, vec2, result);
    std::cout << "Test vectorCrossProduct: Expected {0.0, 0.0, 1.0}, Got {"
              << result[0] << ", " << result[1] << ", " << result[2] << "}" << std::endl;
}

void testVectorAngleBetween() {
    float vec1[3] = {1.0, 0.0, 0.0};
    float vec2[3] = {0.0, 1.0, 0.0}; // Angle should be 90 degrees (π/2 radians)
    float result = vectorAngleBetween(vec1, vec2);
    std::cout << std::setprecision(4) << "Test vectorAngleBetween: Expected 1.5708 radians, Got " 
              << result << " radians" << std::endl;
}

int main() {
    std::cout << "Running tests" << std::endl;
    
    testVectorMag();
    testVectorDotProduct();
    testVectorCrossProduct();
    testVectorAngleBetween();
    
    std::cout << "All tests completed." << std::endl;
    return 0;
}
