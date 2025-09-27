#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <array>
#include <stdexcept>
#include <tuple> 
#include <cmath>
namespace SimCore{



// ellipsoidalClamp:
// Clamps a 3D point (x, y, z) to remain within an axis-aligned ellipsoid centered at the origin.
// The ellipsoid is defined by:
//   - radius a along the x-axis
//   - radius c along the y-axis
//   - radius b along the z-axis
// If the point lies outside this ellipsoid, it is uniformly scaled toward the origin until it lies
// exactly on the ellipsoid’s surface, preserving direction. This allows enforcing directionally
// asymmetric spatial limits (non-uniform clamping bounds).
template<typename T>
std::tuple<T, T, T> ellipsoidalClamp(T x, T y, T z, T a, T c, T b) {
    // Compute normalized squared length inside ellipsoidal space
    T ellipsoidLengthSq = (x * x) / (a * a) + (y * y) / (c * c) + (z * z) / (b * b);

    // Inside ellipsoid — no clamping needed
    if (ellipsoidLengthSq <= 1.0f) {
        return {x, y, z};
    }

    // Clamp by scaling down to ellipsoid surface
    T scale = 1.0f / std::sqrt(ellipsoidLengthSq);
    return {x * scale, y * scale, z * scale};
}


// ellipsoidalClamp2D:
// Clamps a 2D point (x, y) to remain within an axis-aligned ellipse centered at the origin.
// The ellipse is defined by:
//   - radius a along the x-axis
//   - radius b along the y-axis
// If the point lies outside this ellipse, it is scaled uniformly toward the origin
// so it lies exactly on the ellipse boundary, preserving direction.
// This allows enforcing asymmetric elliptical bounds rather than square or circular limits.
template<typename T>
std::pair<T, T> ellipsoidalClamp2D(T x, T y, T a, T b) {
    // Compute normalized squared length in elliptical space
    T ellipseLengthSq = (x * x) / (a * a) + (y * y) / (b * b);

    // Inside ellipse — no clamping needed
    if (ellipseLengthSq <= T(1)) {
        return {x, y};
    }

    // Clamp by scaling to ellipse boundary
    T scale = T(1) / std::sqrt(ellipseLengthSq);
    return {x * scale, y * scale};
}


// circularClamp:
// Clamps the 2D vector (x, y) to lie within a circle of radius maxRadius centered at the origin.
// If the vector’s magnitude exceeds maxRadius, it is scaled back to lie on the circle's edge.
// This enforces a radial boundary rather than clamping each axis independently (square clamp).
template<typename T>
std::tuple<T, T, T> sphericalClamp(T x, T y, T z, T maxRadius) {
    T length = std::sqrt(x * x + y * y + z * z);

    // If within the sphere, no clamping needed
    if (length <= maxRadius) {
        return {x, y, z};
    }

    // Scale down to the sphere surface
    T scale = maxRadius / length;

    return {x * scale, y * scale, z * scale};
}
// sphericalClamp:
// Clamps the 3D vector (x, y, z) to lie within a sphere of radius maxRadius centered at the origin.
// If the vector’s magnitude exceeds maxRadius, it is scaled back to lie on the sphere's surface.
// This preserves direction while enforcing a spherical constraint, unlike axis-wise box clamping.
template<typename T>
inline std::pair<T, T> circularClamp(T x, T y, T maxRadius) {
    T length = std::sqrt(x * x + y * y);

    // If within the circle, no clamping needed
    if (length <= maxRadius) {
        return {x, y};
    }

    // Scale down to the circle edge
    T scale = maxRadius / length;
    return {x * scale, y * scale};
}

template <typename T,std::size_t N>
T vectorMag(const std::array<T,N> &vector){
    T sum = 0;
    for(std::size_t i = 0 ; i < N ; i++){
        sum += vector[i] * vector[i];
    }
    return std::sqrt(sum);
}


template <typename T,size_t N>
T vectorDotProduct(const std::array<T,N> &vector1,const std::array<T,N> &vector2) {
    T dotProduct = T(0.0);

    //v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

    for (size_t i = 0; i < N; i++) {
        dotProduct += vector1[i] * vector2[i];
    }

    return dotProduct;
}


template <typename T>
void vectorCrossProduct(const std::array<T,3> &vector1,const std::array<T,3> &vector2, std::array<T,3> &result) {

    result[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1]; 
    result[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2]; 
    result[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0]; 
}


template <typename T>
T vectorAngleBetween(const std::array<T,3> &vector1,const std::array<T,3> &vector2) {
    
    T mag1 = vectorMag(vector1);
    T mag2 = vectorMag(vector2);
    T dot = vectorDotProduct(vector1, vector2);

  
    if(mag1 == 0 || mag2 == 0) return 0;
    T dotOverMags = dot / (mag1 * mag2);
    if(dotOverMags < -1) return 3.1415;
    if(dotOverMags > 1) return 0;
    return acos(dotOverMags); // Result is in radians
}

template <typename T>
void normalizeVectorInPlace(std::array<T,3>& vector1) {
    T mag = vectorMag(vector1);
    if (mag == 0) {
        vector1 = {0, 0, 0};
        return;
    }
    for (int i = 0; i < 3; i++) {
        vector1[i] /= mag;
    }
}


template <typename T>
std::array<T,3> normalizeVector(const std::array<T,3> vector1){
    std::array<T,3> normalVector;
    T mag = vectorMag(vector1);
    if(mag == 0) return normalVector = {0,0,0};
    for(int i = 0 ; i< 3 ; i++) {
        normalVector[i] = vector1[i]/mag;
    }
    return normalVector;
}


template <typename T>
T twodAngleDiffrence(const std::array<T , 2> &vector1 ,const std::array<T , 2> &vector2){

    T vectDot = vector1[0] * vector2[0] + vector1[1] * vector2[1];

    T mag1 = sqrtf(vector1[0] * vector1[0] + vector1[1] * vector1[1]);
    T mag2 = sqrtf(vector2[0] * vector2[0] + vector2[1] * vector2[1]);

    if(mag1 == 0 || mag2 == 0) return 0;

    T insideCos = (vectDot)/(mag1 * mag2);

    if(insideCos > 1){
        insideCos = 1;
    }else if(insideCos < -1){
        insideCos = -1;
    }

    T angle = acosf(insideCos);

    T cross = (vector1[0] * vector2[1] - vector1[1] * vector2[0]);
    
    if(cross < 0){
        return -angle;
    }
    return angle;

}


template <typename T>
std::array<T , 3> addVectors(const std::array<T , 3> &vec1 ,const std::array<T , 3> &vec2){
    std::array<T , 3> addedVector= {vec1[0]+vec2[0],vec1[1]+vec2[1],vec1[2]+vec2[2]};
    return addedVector;
}


template <typename T>
bool directionality(std::array<T , 3> &refranceVector ,std::array<T , 3> &vector1){

    std::array<T , 3> normalRefranceVector = normalizeVector(refranceVector);
    std::array<T , 3> normalVector1 = normalizeVector(vector1);

    T sqrtTwo = 1.414213562;
    std::array<T,3> sumOfVect = addVectors(normalRefranceVector,normalVector1);
    T chordLength = vectorMag(sumOfVect);

    if( chordLength > sqrtTwo) return false;
    return true;
}


template <typename T>
void setInBounds(T &value , const T &lower , const T &upper){
    
    if(value > upper) value = upper;
    if(value < lower) value = lower;

}

template <typename T>
inline bool isZeroVector(T &vector){
    if(vector[0] == 0 && vector[1] == 0 && vector[2] == 0) return true;
    return false;
}

template <typename T, typename X>
inline void resizeVectorIfLarger(T& vector, X length) {
    X magnitude = 0;
    for (auto& element : vector) {
        magnitude += element * element;
    }
    magnitude = std::sqrt(magnitude);

    if (magnitude == 0 || magnitude <= length) {
        return;
    }

    X scale = length / magnitude;
    for (auto& element : vector) {
        element *= scale;
    }
}

template<typename T>
T signedAngle(std::array<T,2> v1, std::array<T,2> v2 = {0,1}) {
    T dot = vectorDotProduct(v1,v2);
    T cross = v1[0]*v2[1] - v1[1]*v2[0];
    return std::atan2(cross, dot);
}

}

#endif