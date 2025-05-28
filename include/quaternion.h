#ifndef QUATERNION_H
#define QUATERNION_H
#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#pragma once
using namespace std;

namespace SimCore{
// https://en.wikipedia.org/wiki/Quaternion
//https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
// Quaternions: black magic that actually works.
// Rotates vectors in 3D using 4D math, avoids gimbal lock and interpolates smoothly.

// rotation matrix version experiences gimbal lock.


class Quaternion {
public:
    float w, x, y, z;

    Quaternion(float w, float x, float y, float z);

    Quaternion conjugate() const;

    Quaternion operator*(const Quaternion& quat) const;

   inline Quaternion normalized(){
    float norm = std::sqrt(w*w + x*x + y*y + z*z);
    return Quaternion(w / norm, x / norm, y / norm, z / norm);
    }

};

//const Quaternion to make it immutable so that it can be used multiple times one more than 1 vectors.
std::array<float, 3> rotateVector(const Quaternion& q, const std::array<float, 3>& v);


Quaternion fromAxisAngle(std::array<float, 3> axis, float angle_rad);


class quaternionVehicle{
    private:
    std::array<float,3> dirVector;
    std::array<float,3> fwdVector;
    std::array<float,3> rightVector;
    int numberOfCalls;
    public:
    //start direction
    quaternionVehicle(std::array<float,3> dirVectorInit , std::array<float,3> fwdVectorInit);
    //update direction and foward vector
    void eularRotation(float rotationInRadsX , float rotationInRadsY ,float rotationInRadsZ);
    //uses the direction vector as the basis for rotation
    void applyYaw(float rotationInRads);
    //Gram-Schmidt orthonormalization
    void orthogonalize(std::array<float,3>& vector1 , std::array<float,3>& vector2);
    
    inline std::array<float,3> getdirVector(){
        return dirVector;
    }
    inline std::array<float,3> getfwdVector(){
        return fwdVector;
    }
    //direction(top) Vector , fwdVector , rightVector
    inline std::array<std::array<float,3>,3> getPose(){
        return {dirVector,
                fwdVector,
                rightVector};
    }
};

}

#endif