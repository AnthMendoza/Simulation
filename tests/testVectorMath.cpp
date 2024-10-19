#include <iostream>
#include <iomanip>
#include <array>
#include "../include/vectorMath.h"



void testTwoDAngles(){
    std::array<float,2> vec1 = {0,1};
    std::array<float,2> vec2 = {-1,-1};

    std::cout<<twodAngleDiffrence(vec1,vec2);
}

int main() {
    std::cout << "Running tests" << std::endl;
    
    //testVectorMag();
    //testVectorDotProduct();
    //testVectorCrossProduct();
    //testVectorAngleBetween();
    //testNormalize();
    testTwoDAngles();
    
    std::cout << "All tests completed." << std::endl;
    return 0;
}
