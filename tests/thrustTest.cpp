#include "../include/rotationMatrix.h"
#include <iostream>

int main(){

    std::array<float , 3> forceVector = {1,0,0};
    // reverses the vector so that we can apply a thrust in the oposite direction
    for(int i = 0; i<3 ; i++){
        forceVector[i] = -forceVector[i]; 
    } 

    Matrix3x3 Xrotation = rotationMatrixX(0.785398f);
    Matrix3x3 yrotation = rotationMatrixY(0.785398f);
    yrotation = yrotation * Xrotation;

    forceVector = yrotation.rotate(forceVector);


    
    std::cout<<forceVector[0]<< " " <<forceVector[1]<< " " <<forceVector[2];

    return 0;
}

