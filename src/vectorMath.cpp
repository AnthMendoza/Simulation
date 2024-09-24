#include <math.h>
#include "../include/vectorMath.h"



float vectorMag(float (&vector1)[3]){
    return sqrt(vector1[0]*vector1[0] + vector1[1]*vector1[1] + vector1[2]*vector1[2]);
}


float vectorDotProduct(float (&vector1)[3], float (&vector2)[3]) {
    float dotProduct = 0.0;

    //v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    for (int i = 0; i < 3; i++) {
        dotProduct += vector1[i] * vector2[i];
    }

    return dotProduct;
}


void vectorCrossProduct(float (&vector1)[3], float (&vector2)[3], float (&result)[3]) {
    result[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1]; 
    result[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2]; 
    result[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0]; 
}




float vectorAngleBetween(float (&vector1)[3], float (&vector2)[3]) {
    
    float mag1 = vectorMag(vector1);
    float mag2 = vectorMag(vector2);
    
    float dot = vectorDotProduct(vector1, vector2);
    

    return acos(dot / (mag1 * mag2)); // Result is in radians
}