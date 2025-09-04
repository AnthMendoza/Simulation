#include "../include/forceApplied.h"
#include <array>
#include <iostream>

using namespace std;


int main(){
    array<float,3> force = {0,0,1};
    array<float,3> vehicleState = {1,0,0};
    float distance = -3;
    array<float,3> vect = forceToMoment(force , vehicleState ,distance);
    cout <<"expected (-3,6,-3) : "<<vect[0]<< " " <<vect[1] << " " <<vect[2];
}