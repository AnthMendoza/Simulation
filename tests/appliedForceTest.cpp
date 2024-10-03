#include "../include/forceApplied.h"
#include <array>
#include <iostream>

using namespace std;


int main(){
    array<float,3> force = {5,6,7};
    array<float,3> vehicleState = {2,3,4};
    float distance = 5.3851648;
    array<float,3> vect = forceToMoment(force , vehicleState ,distance);
    cout <<"expected (-3,6,-3) : "<<vect[0]<< " " <<vect[1] << " " <<vect[2];
}