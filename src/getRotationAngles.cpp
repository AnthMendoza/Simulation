#include <iostream>
#include <cmath>
#include "../include/Eigen/Eigen"
#include "../include/Eigen/Dense"

Eigen::Vector3d getRotationAngles(const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
    Eigen::Vector3d v = from.cross(to);
    double s = v.norm();
    double c = from.dot(to);

    Eigen::Matrix3d vx;
    vx << 0, -v(2), v(1),
          v(2), 0, -v(0),
         -v(1), v(0), 0;

    Eigen::Matrix3d R;
    if (s < 1e-6) {
        // Vectors are parallel
        if (c > 0) {
            R = Eigen::Matrix3d::Identity();
        } else {
            // Vectors are anti-parallel
            R = -Eigen::Matrix3d::Identity();
            R(2, 2) = 1;  // Rotate 180 degrees around any axis perpendicular to 'from'
        }
    } else {
        R = Eigen::Matrix3d::Identity() + vx + (vx * vx) * ((1 - c) / (s * s));
    }

    Eigen::Vector3d angles = R.eulerAngles(0, 1, 2);

    return angles;
}