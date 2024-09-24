#include <math.h>


//float Vectoradd(int a, int b);
//float Vectorsubtract(int a, int b);
//float Vectormultiply(int a, int b);
//float Vectordivide(int a, int b);
//float VectordotProduct();
//float VectorcrossProduct();
//float VectorangleBetween();


float vectorMag(int (&vector1)[3]){
    return sqrt(vector1[0]*vector1[0] + vector1[1]*vector1[1] + vector1[2]*vector1[2]);
}