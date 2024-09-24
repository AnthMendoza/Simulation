#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <stdbool.h>  // Include for bool type if needed
#include <math.h>
#include <thread>
#include <algorithm> 
#include "../include/vectorMath.h"





int main(){
    float vec[3] = {1,1,1};
    printf("%f",vectorMag(vec));
    return 0;
}