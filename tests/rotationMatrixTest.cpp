#include <iostream>
#include <array>
#include "../include/rotationMatrix.h"

using namespace std;

int main() {

    std::array<float, 3> vector = {1.0f, 0.0f, 0.0f};

  
    float angleX = 90.0f; 
    float angleY = 90.0f; 
    float angleZ = 90.0f; 

    
    Matrix3x3 rotX = rotationMatrixX(angleX);
    Matrix3x3 rotY = rotationMatrixY(angleY);
    Matrix3x3 rotZ = rotationMatrixZ(angleZ);

    std::array<float, 3> rotatedX = rotX.rotate(vector);
    std::array<float, 3> rotatedY = rotY.rotate(vector);
    std::array<float, 3> rotatedZ = rotZ.rotate(vector);

    cout << "Original Vector: (" << vector[0] << ", " << vector[1] << ", " << vector[2] << ")\n";
    
    cout << "Rotated Vector around X-axis by " << angleX << " degrees: ("
         << rotatedX[0] << ", " << rotatedX[1] << ", " << rotatedX[2] << ")\n";
    
    cout << "Rotated Vector around Y-axis by " << angleY << " degrees: ("
         << rotatedY[0] << ", " << rotatedY[1] << ", " << rotatedY[2] << ")\n";
    
    cout << "Rotated Vector around Z-axis by " << angleZ << " degrees: ("
         << rotatedZ[0] << ", " << rotatedZ[1] << ", " << rotatedZ[2] << ")\n";
    
    return 0;
}
