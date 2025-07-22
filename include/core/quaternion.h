#ifndef QUATERNION_H
#define QUATERNION_H
#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#include "vectorMath.h"

#pragma once
using namespace std;

namespace SimCore{

using threeDState = std::array<float,3>;

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

//rotate an array(vector) full of vectors.
inline void rotateMultiVectors(const Quaternion& q, vector<std::array<float, 3>>& v){
    for(int i = 0 ; i < v.size() ; i++){
        v[i] = rotateVector(q,v[i]);
    }
}


Quaternion fromAxisAngle(const std::array<float, 3> axis, float angle_rad);

struct poseState{
    std::array<float,3> dirVector;
    std::array<float,3> fwdVector;
    std::array<float,3> rightVector;
};


class quaternionVehicle{
    private:
    poseState pose;
    int numberOfCalls;
    public:
    //start direction
    quaternionVehicle();
    //Right Vector will be calculated using the right hand rule.
    void setVehicleQuaternionState(threeDState dir, threeDState fwd);
    //update direction and foward vector
    Quaternion eularRotation(float rotationInRadsX , float rotationInRadsY ,float rotationInRadsZ);
    //uses the direction vector as the basis for rotation
    void applyYaw(float rotationInRads);
    //Gram-Schmidt orthonormalization
    void orthogonalize(std::array<float,3>& vector1 , std::array<float,3>& vector2);
    
    inline std::array<float,3> getdirVector(){
        return pose.dirVector;
    }
    inline std::array<float,3> getfwdVector(){
        return pose.fwdVector;
    }
    //direction(top) Vector , fwdVector , rightVector
    inline poseState getPose(){
        poseState packet;
        packet.dirVector = pose.dirVector;
        packet.fwdVector = pose.fwdVector;
        packet.rightVector = pose.rightVector;
        return packet;
    }
};

struct rotation{
    threeDState axisOfRotation;
    float angle;
};



class vehicleRefranceFrame{
    private:
    poseState pose;
    poseState basis;
    std::vector<rotation> rotations;
    static constexpr float EPSILON = 1e-4;
    void getQuaternionRotationState(){
        rotations.clear();
        float dotProduct = vectorDotProduct(pose.dirVector, basis.dirVector);
        if (std::fabs(dotProduct + 1) < EPSILON){
            float angle = M_PI;
            threeDState axisOfRotation;
            pose.dirVector = normalizeVector(pose.dirVector);
            if (std::fabs(pose.dirVector[0]) < 0.9f) {
                vectorCrossProduct({1.0f, 0.0f, 0.0f}, pose.dirVector, axisOfRotation);
            } else {
                vectorCrossProduct({0.0f, 1.0f, 0.0f}, pose.dirVector, axisOfRotation);
            }
            normalizeVector(axisOfRotation);

            rotation rotatePack;
            rotatePack.axisOfRotation = axisOfRotation;
            rotatePack.angle = angle;
            rotations.push_back(rotatePack);
    
        }else if(std::fabs(dotProduct - 1) > EPSILON) {
            float angle = vectorAngleBetween(pose.dirVector, basis.dirVector);
            threeDState axisOfRotation;
            vectorCrossProduct(pose.dirVector, basis.dirVector, axisOfRotation);

        
            Quaternion testQuat = fromAxisAngle(axisOfRotation, angle);
            threeDState rotatedPose = rotateVector(testQuat, pose.dirVector);
            dotProduct = vectorDotProduct(rotatedPose, basis.dirVector);
            if(std::fabs(dotProduct - 1) > EPSILON) {
                angle = -angle;
            }

            rotation rotatePack;
            rotatePack.axisOfRotation = axisOfRotation;
            rotatePack.angle = angle;
            rotations.push_back(rotatePack);
        }

        
        threeDState rotatedFwd = pose.fwdVector;
        if(!rotations.empty()) {
            Quaternion firstQuat = fromAxisAngle(rotations[0].axisOfRotation, rotations[0].angle);
            rotatedFwd = rotateVector(firstQuat, rotatedFwd);
        }

        dotProduct = vectorDotProduct(rotatedFwd, basis.fwdVector);
        if(std::fabs(dotProduct - 1) > EPSILON) {
            float angle = vectorAngleBetween(rotatedFwd, basis.fwdVector);

            
            Quaternion testQuat2 = fromAxisAngle(basis.dirVector, angle);
            threeDState testRotated = rotateVector(testQuat2, rotatedFwd);
            dotProduct = vectorDotProduct(testRotated, basis.fwdVector);
            if(std::fabs(dotProduct - 1) > EPSILON) {
                angle = -angle;
            }

            rotation rotatePack;
            rotatePack.axisOfRotation = basis.dirVector;
            rotatePack.angle = angle;
            rotations.push_back(rotatePack); 
        }
    }


    threeDState realignHandler(const threeDState& vec){
        threeDState realigned = vec; 
        for(auto& rotatePack : rotations){
            Quaternion quant = fromAxisAngle(rotatePack.axisOfRotation, rotatePack.angle);
            realigned = rotateVector(quant, realigned);
        }
        return realigned;
    }

    public:

    vehicleRefranceFrame(const poseState& pose,poseState basisVector = {{0.0f,0.0f,1.0f},{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f}}): pose(pose) , basis(basisVector){
        getQuaternionRotationState();
    }

    inline std::vector<threeDState> realign(const std::vector<threeDState>& vecs){
        std::vector<threeDState> realigned;
        for(auto& vec:vecs){
            realigned.push_back(realignHandler(vec));
        }
        return realigned;
    }

    inline threeDState realign(const threeDState& vec){
        return realignHandler(vec);
    }

    inline poseState getBasisPose(){
        return basis;
    }

};


}

#endif