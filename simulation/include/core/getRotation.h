#ifndef GETROTATION_H
#define GETROTATION_H
#include "../thirdparty/eigenWrapper.h"
namespace SimCore{
Eigen::Vector3d getRotationAngles(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
}

#endif 