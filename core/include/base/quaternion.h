#ifndef QUATERNION_H
#define QUATERNION_H
#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#include <optional>
#include <base/units.h>
#include "vectorMath.h"
#include "../util/utility.h"
#include "coordinateSystem.h"
#include "poseState.h"
#include "vectorMath.h"

// *** changed all key words from pose to localPose. unreal engine conflicts
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
    units::scalar w, x, y, z;

    Quaternion(units::scalar w, units::scalar x, units::scalar y, units::scalar z);

    Quaternion conjugate() const;

    Quaternion operator*(const Quaternion& quat) const;

   [[nodiscard]]inline Quaternion normalized() const {
    units::scalar norm = std::sqrt(w*w + x*x + y*y + z*z);
    if(norm < 1e-8f) { 
        return Quaternion(1, 0, 0, 0);
    }
    return Quaternion(w / norm, x / norm, y / norm, z / norm);
    }

};

//const Quaternion to make it immutable so that it can be used multiple times one more than 1 vectors.
std::array<units::scalar, 3> rotateVector(const Quaternion& q, const std::array<units::scalar, 3>& v);

//rotate an array(vector) full of vectors.
inline void rotateMultiVectors(const Quaternion& q, vector<std::array<units::scalar, 3>>& v){
    for(int i = 0 ; i < v.size() ; i++){
        v[i] = rotateVector(q,v[i]);
    }
}


Quaternion fromAxisAngle(const std::array<units::scalar, 3> axis, units::scalar angle_rad);



class quaternionVehicle{
    private:
    poseState localPose;
    int numberOfCalls;
    public:
    //start direction
    quaternionVehicle();
    //Right Vector will be calculated using the right hand rule.
    void setVehicleQuaternionState(units::vec3 dir, units::vec3 fwd);
    //update direction and foward vector
    Quaternion eulerRotation(units::scalar rotationInRadsX , units::scalar rotationInRadsY ,units::scalar rotationInRadsZ);
    //uses the direction vector as the basis for rotation
    void applyYaw(units::scalar rotationInRads);
    //Gram-Schmidt orthonormalization
    void orthogonalize(std::array<units::scalar,3>& vector1 , std::array<units::scalar,3>& vector2);

    void rotatePose(const Quaternion& quant);
    
    inline std::array<units::scalar,3> getdirVector(){
        return localPose.dirVector;
    }
    inline std::array<units::scalar,3> getfwdVector(){
        return localPose.fwdVector;
    }
    //direction(top) Vector , fwdVector , rightVector
    inline poseState getPose(){
        poseState packet;
        packet.dirVector = localPose.dirVector;
        packet.fwdVector = localPose.fwdVector;
        packet.rightVector = localPose.rightVector;
        return packet;
    }
};

struct rotation{
    units::vec3 axisOfRotation;
    units::scalar angle;
};

static bool similarVector(const units::vec3& vec1 , const units::vec3& vec2 ,units::scalar EPSILON = 1e-4){
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


class vehicleReferenceFrame{
    private:
    std::optional<poseState> localPose;
    std::optional<poseState> basis;
    std::vector<rotation> rotations;
    static constexpr units::scalar EPSILON = 1e-4;

void getQuaternionRotationState(){
    if(!localPose){
        std::cout<<"Warning localPose not initialized in vehicleReferenceFrame.\n";
        return;
    }
    if(!basis){
        std::cout<<"Warning basis not initialized in vehicleReferenceFrame.\n";
        return;
    }
    if(arePosesEqual(*localPose,*basis)){
        rotations.clear();
        return;
    }
    
    rotations.clear();
    localPose->normalize();
    basis->normalize();

    poseState progressPose = *localPose;
    
    units::scalar dotProduct = vectorDotProduct(localPose->dirVector, basis->dirVector);
    if(std::fabs(dotProduct + 1) < EPSILON){
        rotation rot;
        rot.angle = M_PI;
        rot.axisOfRotation = localPose->fwdVector;
        rotations.push_back(rot);
    }
    //rotation.size() == 0 is just checking if the previous if statement added a rotation alread. if it did. the next step is covered.
    if(std::fabs(dotProduct - 1) > EPSILON && rotations.size() == 0){
        units::vec3 rotationAxis;
        vectorCrossProduct(localPose->dirVector,basis->dirVector,rotationAxis);
        normalizeVectorInPlace(rotationAxis);
        units::scalar angle = vectorAngleBetween(localPose->dirVector,basis->dirVector);
        Quaternion quant = fromAxisAngle(rotationAxis,angle);
        units::vec3 rotateTest = rotateVector(quant,localPose->dirVector);

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
        units::scalar angle = vectorAngleBetween(progressPose.fwdVector,basis->fwdVector);
        Quaternion quant = fromAxisAngle(basis->dirVector,angle);
        units::vec3 rotateTest = rotateVector(quant,progressPose.fwdVector);
        if(!similarVector(rotateTest,basis->fwdVector,EPSILON)){
            angle = -angle;
        }
        rotation rot;
        rot.angle = angle;
        rot.axisOfRotation = basis->dirVector;
        rotations.push_back(rot);

    }

    
}


    units::vec3 realignHandler(const units::vec3& vec){
        units::vec3 realigned = vec; 
        for(auto& rotatePack : rotations){
            Quaternion quant = fromAxisAngle(rotatePack.axisOfRotation, rotatePack.angle);
            realigned = rotateVector(quant, realigned);
        }
        return realigned;
    }

    public:

    vehicleReferenceFrame(const poseState& localPose,const poseState& basisVector = CoordinateSystem::WORLD_BASIS): localPose(localPose) , basis(basisVector){
        getQuaternionRotationState();
    }

    inline std::vector<units::vec3> realign(const std::vector<units::vec3>& vecs){
        std::vector<units::vec3> realigned;
        for(auto& vec:vecs){
            realigned.push_back(realignHandler(vec));
        }
        return realigned;
    }

    inline units::vec3 realign(const units::vec3& vec){
        return realignHandler(vec);
    }

    inline poseState getBasisPose(){
        return *basis;
    }

    inline void realignPose(poseState &alignmentPose){
        for(auto& rotatePack : rotations){
            Quaternion quant = fromAxisAngle(rotatePack.axisOfRotation, rotatePack.angle);
            alignmentPose.dirVector = rotateVector(quant, alignmentPose.dirVector);
            alignmentPose.fwdVector = rotateVector(quant, alignmentPose.fwdVector);
            normalizeVectorInPlace(alignmentPose.dirVector);
            normalizeVectorInPlace(alignmentPose.fwdVector);
            vectorCrossProduct(alignmentPose.dirVector, alignmentPose.fwdVector, alignmentPose.rightVector);
            normalizeVectorInPlace(alignmentPose.rightVector);

        }
        normalizeVectorInPlace(alignmentPose.dirVector);
        vectorCrossProduct(alignmentPose.dirVector, alignmentPose.fwdVector, alignmentPose.rightVector);
        normalizeVectorInPlace(alignmentPose.rightVector);
        vectorCrossProduct(alignmentPose.rightVector, alignmentPose.dirVector, alignmentPose.fwdVector);
        normalizeVectorInPlace(alignmentPose.fwdVector);
    }

};


}

#endif