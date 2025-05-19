#include <cmath>
#include <array>
#include <iostream>
#include "../include/rotationMatrix.h"
#include "../include/vectorMath.h"

using namespace std;
namespace SimCore{
Matrix3x3::Matrix3x3() {
    m = {{{1.0f, 0.0f, 0.0f},
          {0.0f, 1.0f, 0.0f}, 
          {0.0f, 0.0f, 1.0f}}};
}

// this allows us to multiply matracies 
Matrix3x3 Matrix3x3::operator*(const Matrix3x3& other) const {
    Matrix3x3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = m[i][0] * other.m[0][j] + m[i][1] * other.m[1][j] + m[i][2] * other.m[2][j];
        }
    }
    return result;
}

std::array<float, 3> Matrix3x3::rotate(const std::array<float, 3>& v) const {
    std::array<float, 3> result;
    for (int i = 0; i < 3; ++i) {
        result[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
    }
    return result;
}

void Matrix3x3::printMatrix() const {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout << m[i][j] << " ";
        }
        std::cout << std::endl;
    }
}


// Rotation matrix for rotation around X-axis
Matrix3x3 rotationMatrixX(float angleRadians) {

    

    Matrix3x3 rotation;

    rotation.m[1][1] = cos(angleRadians);
    rotation.m[1][2] = -sin(angleRadians);
    rotation.m[2][1] = sin(angleRadians);
    rotation.m[2][2] = cos(angleRadians);

    return rotation;
}

// Rotation matrix for rotation around Y-axis
Matrix3x3 rotationMatrixY(float angleRadians) {

    Matrix3x3 rotation;

    rotation.m[0][0] = cos(angleRadians);
    rotation.m[0][2] = sin(angleRadians);
    rotation.m[2][0] = -sin(angleRadians);
    rotation.m[2][2] = cos(angleRadians);

    return rotation;
}

// Rotation matrix for rotation around Z-axis
Matrix3x3 rotationMatrixZ(float angleRadians) {

    Matrix3x3 rotation;

    rotation.m[0][0] = cos(angleRadians);
    rotation.m[0][1] = -sin(angleRadians);
    rotation.m[1][0] = sin(angleRadians);
    rotation.m[1][1] = cos(angleRadians);

    return rotation;
}

}












