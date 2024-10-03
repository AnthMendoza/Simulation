#ifndef ROTATIONMATRIX_H
#define ROTATIONMATRIX_H

#include <array>


class Matrix3x3 {
public:
    std::array<std::array<float, 3>, 3> m;

    Matrix3x3(); 

    Matrix3x3 operator*(const Matrix3x3& other) const;

    std::array<float, 3> rotate(const std::array<float, 3>& v) const;

    void printMatrix() const;
};


Matrix3x3 rotationMatrixX(float angleRadians);
Matrix3x3 rotationMatrixY(float angleRadians);
Matrix3x3 rotationMatrixZ(float angleRadians);


#endif