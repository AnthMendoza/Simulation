#include "../include/quaternion.h"
#include "../include/vectorMath.h"
#include <cmath>

namespace SimCore{
//w is just a scaler of vector x,y,z
Quaternion::Quaternion(float w, float x, float y, float z): w(w), x(x), y(y), z(z) {} 

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

std::array<float, 3> rotateVector(const Quaternion& q, const std::array<float, 3>& v) {
    Quaternion v_q(0.0f, v[0], v[1], v[2]);
    Quaternion q_conj = q.conjugate();
    Quaternion rotated_q = q * v_q * q_conj;
    return {rotated_q.x, rotated_q.y, rotated_q.z};
}

Quaternion fromAxisAngle(std::array<float, 3> axis, float angle_rad) {
    float half_angle = angle_rad / 2.0f;
    float s = sin(half_angle);
    return Quaternion{
        cos(half_angle),
        axis[0] * s,
        axis[1] * s,
        axis[2] * s
    };
}

constexpr float EPSILON = 1e-4;
quaternionVehicle::quaternionVehicle(std::array<float,3> dirVectorInit , std::array<float,3> fwdVectorInit){
    if(std::abs(vectorDotProduct(dirVectorInit,fwdVectorInit)) < EPSILON){
        dirVector = dirVectorInit;
        fwdVector = fwdVectorInit;
    }else{

        std::cerr<<"Warning inital state is not valid as the forward and direction vector are not orthogonal. Setting Default values.";
    }
}
//eular rotaion, rotates around x,y,z axis.
void quaternionVehicle::eularRotation(float rotationInRadsX , float rotationInRadsY ,float rotationInRadsZ){
    ++numberOfCalls;
    Quaternion qx = fromAxisAngle({1,0,0}, rotationInRadsX);
    Quaternion qy = fromAxisAngle({0,1,0}, rotationInRadsY);
    Quaternion qz = fromAxisAngle({0,0,1}, rotationInRadsZ);


    Quaternion combined = qz * qy * qx;
    combined = combined.normalized();

    dirVector = rotateVector(combined, dirVector);
    fwdVector = rotateVector(combined, fwdVector);
    // call for Gram-Schmidt orthonormalization
    if(numberOfCalls > 20){
        orthogonalize();
        numberOfCalls = 0;
    }

}

void quaternionVehicle::applyYaw(float rotationInRads){
    Quaternion rotationQuat = fromAxisAngle(normalizeVector(dirVector),rotationInRads);
    fwdVector = rotateVector(rotationQuat,fwdVector);
} 

void quaternionVehicle::orthogonalize(){
    // Normalize v1
    dirVector = normalizeVector(dirVector);

    // Project v2 onto v1 and subtract to make v2 perpendicular to v1
    float dotProd = vectorDotProduct(dirVector, fwdVector);
    std::array<float, 3> proj = {
        dotProd * dirVector[0],
        dotProd * dirVector[1],
        dotProd * dirVector[2]
    };

    fwdVector = {
        fwdVector[0] - proj[0],
        fwdVector[1] - proj[1],
        fwdVector[2] - proj[2]
    };

    // Normalize v2
    fwdVector = normalizeVector(fwdVector);
}



}