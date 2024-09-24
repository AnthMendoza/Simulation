#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

float vectorMag(float (&vector)[3]);

float vectorDotProduct(float (&vector1)[3], float (&vector2)[3]);

void vectorCrossProduct(float (&vector1)[3], float (&vector2)[3], float (&result)[3]);

float vectorAngleBetween(float (&vector1)[3], float (&vector2)[3]);


#endif