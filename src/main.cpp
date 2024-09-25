#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <stdbool.h>  
#include <math.h>
#include <thread>
#include <algorithm> 
#include "../include/vectorMath.h"
#include "../include/vehicle.h"



int main(){
    float val[3] = {1,2,4};
    Vehicle myVehicle(0.0f, 1.0f, 2.0f, 0.5f, 0.1f, 1.57f, val);
    myVehicle.display();
    return 0;
}