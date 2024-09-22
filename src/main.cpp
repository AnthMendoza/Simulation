#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>  // Include for bool type if needed
#include <math.h>
#include <thread>
#include <algorithm> 



float lengthOfVector(float *vector){
    return sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2] );
}
float angleOfTwoVectors(float *vector1, float *vector2){
    float vector1Length = lengthOfVector(vector1);
    float vector2Length = lengthOfVector(vector2);
    if(vector1Length == 0 || vector2Length == 0 ){
        return 0;
    }
    return (acos(vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2]))/(vector1Length * vector2Length);
}

void pr(float *vector1, float *vector2){
    printf("%f\n",angleOfTwoVectors(vector1,vector2));
}




int main(){
    float vect1[3] = {10,2,0};
    float vect2[3] = {5,3,0};
    
    std::thread xthread(pr,vect1,vect2);
    std::thread ythread(pr,vect1,vect2);
    std::thread zthread(pr,vect1,vect2);
    
    xthread.join();
    ythread.join();
    zthread.join();
    return 0;
}