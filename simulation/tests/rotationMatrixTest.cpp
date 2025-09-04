#include <iostream>
#include <array>
#include "../include/rotationMatrix.h"

using namespace std;

int main() {

    std::array<float, 3> vector = {1.0f, 0.0f, 0.0f};

  
    float angleX =  3.11342f; 
    float angleY =  3.00301f; 
    float angleZ = -2.74112f; 

    
    Matrix3x3 rotX = rotationMatrixX(angleX);
    Matrix3x3 rotY = rotationMatrixY(angleY);
    Matrix3x3 rotZ = rotationMatrixZ(angleZ);

     Matrix3x3 combined = rotX * rotY * rotZ;

     std::array<float,3> vector2 = combined.rotate(vector);

    cout << " " << vector2[0] << ", " <<  vector2[1] << ", " << vector2[2] << "\n";
    

    
    return 0;
}
