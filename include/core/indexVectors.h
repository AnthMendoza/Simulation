#ifndef INDEXVECTOR_H
#define INDEXVECTOR_H
#include <array>
namespace SimCore{
struct nominalVectors {
    const std::array<float, 3> dir = {0, 0, 1};
    const std::array<float, 3> fwd = {1, 0, 0};
    const std::array<float, 3> right = {0, 1, 0};
};
//each struct has has a refrance
struct indexCoordinates {
    nominalVectors nominal;

    std::array<float, 3> dirVector;
    std::array<float, 3> fwdVector;
    std::array<float, 3> rightVector;

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