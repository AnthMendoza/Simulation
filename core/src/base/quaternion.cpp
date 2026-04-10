#include "../include/base/quaternion.h"
#include "../include/base/vectorMath.h"
#include "../include/base/coordinateSystem.h"
#include <cmath>
#include <iostream>
#include <assert.h>
#include <iostream>
#include <base/units.h>

namespace SimCore{
//w is just a scaler of vector x,y,z
Quaternion::Quaternion(units::scalar w, units::scalar x, units::scalar y, units::scalar z): w(w), x(x), y(y), z(z) {} 

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::operator*(const Quaternion& quat) const {
    return Quaternion{
        w * quat.w - x * quat.x - y * quat.y - z * quat.z,
        w * quat.x + x * quat.w + y * quat.z - z * quat.y,
        w * quat.y - x * quat.z + y * quat.w + z * quat.x,
        w * quat.z + x * quat.y - y * quat.x + z * quat.w
    };
}

units::vec3 rotateVector(const Quaternion& q, const units::vec3& v) {
    Quaternion v_q(0.0f, v[0], v[1], v[2]);
    Quaternion q_conj = q.conjugate();
    Quaternion rotated_q = q * v_q * q_conj;
    return {rotated_q.x, rotated_q.y, rotated_q.z};
}

Quaternion fromAxisAngle(const units::vec3 axis, units::scalar angle_rad) {
    units::scalar half_angle = angle_rad / 2.0f;
    units::scalar s = sin(half_angle);
    Quaternion quant{
        cos(half_angle),
        axis[0] * s,
        axis[1] * s,
        axis[2] * s
    };

    quant = quant.normalized();

    return quant;

}

static constexpr units::scalar EPSILON = 1e-4;
quaternionVehicle::quaternionVehicle(): localPose{CoordinateSystem::WORLD_BASIS}, numberOfCalls(0){
    

    //std::cerr<<"Warning inital state is not valid as the forward and direction vector are not orthogonal. Setting Default values.";
}

void quaternionVehicle::setVehicleQuaternionState(units::vec3 dir , units::vec3 fwd){

    localPose.dirVector = normalizeVector(dir);
    localPose.fwdVector = normalizeVector(fwd);

    units::scalar dotProduct = vectorDotProduct(dir,fwd);

    if (std::fabs(dotProduct) >= 1.0f - EPSILON) {
        std::cerr << "Warning: fwdVector is parallel or antiparallel to dirVector. Generating arbitrary orthogonal fwdVector.\n";

        
        units::vec3 arbitrary = {1.0f, 0.0f, 0.0f};
        if (std::fabs(vectorDotProduct(dir, arbitrary)) > 0.99f) {
            arbitrary = {0.0f, 1.0f, 0.0f}; 
        }

        
        units::vec3 orthogonalFwd = {
            dir[1] * arbitrary[2] - dir[2] * arbitrary[1],
            dir[2] * arbitrary[0] - dir[0] * arbitrary[2],
            dir[0] * arbitrary[1] - dir[1] * arbitrary[0]
        };

        normalizeVector(orthogonalFwd);
        localPose.fwdVector = orthogonalFwd;
        return;
    }

    if (std::fabs(dotProduct) >= EPSILON) {
        std::cerr << "Warning: dirVector and fwdVector were not orthogonal when attempting to set in QuaternionVehicle.\n";

        
        units::scalar projection = vectorDotProduct(fwd, dir);
        units::vec3 projectedComponent = {
            projection * dir[0],
            projection * dir[1],
            projection * dir[2]
        };

        units::vec3 orthogonalFwd = {
            fwd[0] - projectedComponent[0],
            fwd[1] - projectedComponent[1],
            fwd[2] - projectedComponent[2]
        };

        orthogonalFwd = normalizeVector(orthogonalFwd);

        localPose.fwdVector = orthogonalFwd;

    }
    vectorCrossProduct(localPose.fwdVector,localPose.dirVector,localPose.rightVector);

}


//eular rotaion, rotates around x,y,z axis.
Quaternion quaternionVehicle::eulerRotation(units::scalar rotationInRadsX , units::scalar rotationInRadsY ,units::scalar rotationInRadsZ){
    ++numberOfCalls;
    Quaternion qx = fromAxisAngle({1,0,0}, rotationInRadsX);
    Quaternion qy = fromAxisAngle({0,1,0}, rotationInRadsY);
    Quaternion qz = fromAxisAngle({0,0,1}, rotationInRadsZ);

    Quaternion combined = qz * qy * qx;
    combined = combined.normalized();

    rotatePose(combined);
    if(numberOfCalls > 100){
        orthogonalize(localPose.dirVector,localPose.fwdVector);
        orthogonalize(localPose.dirVector,localPose.rightVector);
        orthogonalize(localPose.fwdVector,localPose.rightVector);
        numberOfCalls = 0;
    }
    return combined;
}

void quaternionVehicle::applyYaw(units::scalar rotationInRads){
    Quaternion rotationQuat = fromAxisAngle(normalizeVector(localPose.dirVector),rotationInRads);
    localPose.fwdVector = rotateVector(rotationQuat,localPose.fwdVector);
} 

void quaternionVehicle::orthogonalize(units::vec3& vector1 , units::vec3& vector2){
    // Normalize v1
    vector1 = normalizeVector(vector1);

    // Project v2 onto v1 and subtract to make v2 perpendicular to v1
    units::scalar dotProd = vectorDotProduct(vector1, vector2);
    units::vec3 proj = {
        dotProd * vector1[0],
        dotProd * vector1[1],
        dotProd * vector1[2]
    };

    vector2 = {
        vector2[0] - proj[0],
        vector2[1] - proj[1],
        vector2[2] - proj[2]
    };

    // Normalize v2
    vector2 = normalizeVector(vector2);
}

void quaternionVehicle::rotatePose(const Quaternion& quant){
    localPose.dirVector = rotateVector(quant,localPose.dirVector);
    localPose.fwdVector = rotateVector(quant,localPose.fwdVector);
    normalizeVectorInPlace(localPose.dirVector);
    normalizeVectorInPlace(localPose.fwdVector);
    vectorCrossProduct(localPose.dirVector, localPose.fwdVector, localPose.rightVector);
    normalizeVectorInPlace(localPose.rightVector);
}

} 