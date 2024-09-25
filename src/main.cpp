#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <stdbool.h>  
#include <math.h>
#include <thread>
#include <algorithm> 
#include "../include/vectorMath.h"
#include "../include/vehicle.h"
#include "../include/odeIterator.h"



int main(){
    float val[3] = {1,2,4};

    Vehicle myVehicle(0.0f, 0.0f, 0.0f, 0.5f, 0.1f, 1.57f, val);
    Ode(10.0f,5.0f,.05f,myVehicle.Xvelocity, myVehicle.Xposition);
    myVehicle.display();
    return 0;
}