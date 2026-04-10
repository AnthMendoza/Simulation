#ifndef INDEXVECTOR_H
#define INDEXVECTOR_H
#include <array>
namespace SimCore{
struct nominalVectors {
    const units::vec3 dir = {0, 0, 1};
    const units::vec3 fwd = {1, 0, 0};
    const units::vec3 right = {0, 1, 0};
};
//each struct has has a refrance
struct indexCoordinates {
    nominalVectors nominal;

    units::vec3 dirVector;
    units::vec3 fwdVector;
    units::vec3 rightVector;

    indexCoordinates()
        : dirVector(nominal.dir),
          fwdVector(nominal.fwd),
          rightVector(nominal.right) {}
    //copy constructor allows reseting of struct variables
    indexCoordinates& operator=(const indexCoordinates& other) {
        dirVector = other.dirVector;
        fwdVector = other.fwdVector;
        rightVector = other.rightVector;
        return *this;
    }

};
}//SimCore
#endif