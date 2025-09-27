#ifndef QUATERNION_H
#define QUATERNION_H
#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#include <optional>
#include "vectorMath.h"
#include "../utility/utility.h"

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
    void printPose(){
        std::cout << "poseState {\n";
        print(dirVector,"  dirVector");
        print(fwdVector , "  fwdVector");
        print(rightVector, "  rightVector");
        std::cout << "}\n";
    }
    void normalize(){
        normalizeVectorInPlace(dirVector);
        normalizeVectorInPlace(fwdVector);
        normalizeVectorInPlace(rightVector);
    }
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

    void rotatePose(const Quaternion& quant);
    
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

static bool similarVector(const threeDState& vec1 , const threeDState& vec2 ,float EPSILON = 1e-4){
    if(vec1.size() != vec2.size()) return false;
    for(int i = 0; i < vec1.size();  i++){
        if(std::fabs(vec1[i] - vec2[i]) > EPSILON){
            return false;
        }
    }
    return true;
}

static bool arePosesEqual(const poseState& pose1, const poseState& pose2) {
    if(!similarVector(pose1.dirVector,pose2.dirVector)){ return false;}
    if(!similarVector(pose1.fwdVector,pose2.fwdVector)){ return false;}
    if(!similarVector(pose1.rightVector,pose2.rightVector)){ return false;}
    return true;
}


class vehicleRefranceFrame{
    private:
    std::optional<poseState> pose;
    std::optional<poseState> basis;
    std::vector<rotation> rotations;
    static constexpr float EPSILON = 1e-4;

void getQuaternionRotationState(){
    if(!pose){
        std::cout<<"Warning pose not initialized in vehicleRefranceFrame.\n";
        return;
    }
    if(!basis){
        std::cout<<"Warning basis not initialized in vehicleRefranceFrame.\n";
        return;
    }
    if(arePosesEqual(*pose,*basis)){
        rotations.clear();
        return;
    }
    
    rotations.clear();
    pose->normalize();
    basis->normalize();

    poseState progressPose = *pose;
    
    float dotProduct = vectorDotProduct(pose->dirVector, basis->dirVector);
    if(std::fabs(dotProduct + 1) < EPSILON){
        rotation rot;
        rot.angle = M_PI;
        rot.axisOfRotation = pose->fwdVector;
        rotations.push_back(rot);
    }
    //rotation.size() == 0 is just checking if the previous if statement added a rotation alread. if it did. the next step is covered.
    if(std::fabs(dotProduct - 1) > EPSILON && rotations.size() == 0){
        threeDState rotationAxis;
        vectorCrossProduct(pose->dirVector,basis->dirVector,rotationAxis);
        normalizeVectorInPlace(rotationAxis);
        float angle = vectorAngleBetween(pose->dirVector,basis->dirVector);
        Quaternion quant = fromAxisAngle(rotationAxis,angle);
        threeDState rotateTest = rotateVector(quant,pose->dirVector);

        if(!similarVector(rotateTest,basis->dirVector,EPSILON)){
            angle = -angle;
        }
        rotation rot;
        rot.angle = angle;
        rot.axisOfRotation = rotationAxis;
        rotations.push_back(rot);

    }
    //apply rotations to progressPose if any
    for(auto &rotate:rotations){
        Quaternion quant = fromAxisAngle(rotate.axisOfRotation,rotate.angle);
        progressPose.dirVector = rotateVector(quant, progressPose.dirVector);
        progressPose.fwdVector = rotateVector(quant, progressPose.fwdVector);
        progressPose.rightVector = rotateVector(quant, progressPose.rightVector);
    }
    //the aligned axis now becomes the rotaion axis.
    
    dotProduct = vectorDotProduct(progressPose.fwdVector,basis->fwdVector);
    if(std::fabs(dotProduct - 1) > EPSILON){
        float angle = vectorAngleBetween(progressPose.fwdVector,basis->fwdVector);
        Quaternion quant = fromAxisAngle(basis->dirVector,angle);
        threeDState rotateTest = rotateVector(quant,progressPose.fwdVector);
        if(!similarVector(rotateTest,basis->fwdVector,EPSILON)){
            angle = -angle;
        }
        rotation rot;
        rot.angle = angle;
        rot.axisOfRotation = basis->dirVector;
        rotations.push_back(rot);

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
        return *basis;
    }

    inline void realignPose(poseState &pose){
        for(auto& rotatePack : rotations){
            Quaternion quant = fromAxisAngle(rotatePack.axisOfRotation, rotatePack.angle);
            pose.dirVector = rotateVector(quant, pose.dirVector);
            pose.fwdVector = rotateVector(quant, pose.fwdVector);
            pose.rightVector = rotateVector(quant, pose.rightVector);
        }
    }

};


}

#endif