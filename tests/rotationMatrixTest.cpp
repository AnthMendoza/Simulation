#include <iostream>
#include "../include/rotationMatrix.h"



int main() {
    
    std::array<float, 3> vector = {1.0f, 0.0f, 0.0f}; 

    
    Matrix3x3 rotationY = rotationMatrixY(90.0f);
    Matrix3x3 rotationZ = rotationMatrixZ(90.0f);

    
    Matrix3x3 combinedRotation = rotationZ * rotationY;


    std::array<float, 3> rotatedVector = combinedRotation.rotate(vector);

    
    std::cout << "Rotated vector: (" << rotatedVector[0] << ", " << rotatedVector[1] << ", " << rotatedVector[2] << ")" << std::endl;

    return 0;
}
