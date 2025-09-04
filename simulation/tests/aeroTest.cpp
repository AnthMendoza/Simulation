#include "iostream"
#include "../include/aero.h"


int main(){
    float zposition =  24384;
    std::cout<< "look up table estimate :0.04437 kg/m^3. calculated value = "<< airDensity(zposition) << " kg/m^3" << "\n";
    zposition = 152.4;
    std::cout<< "look up table estimate :1.05561674 kg/m^3. calculated value = "<< airDensity(zposition) << " kg/m^3" << "\n";
    zposition = 76200; 
    std::cout<< "look up table estimate :3.34785e-5 kg/m^3. calculated value = "<< airDensity(zposition) << " kg/m^3" << "\n";

    return 0;
}