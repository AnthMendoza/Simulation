#pragma once


#include "quaternion.h"
#include "vectorMath.h"
#include "coordinateSystem.h"
#include <base/units.h>

namespace SimCore {

class transpose {
    private:

    units::vec3 nominalPosition;
    units::vec3 transposePosition;

    poseState nominalState;
    quaternionVehicle transposeState;

    int rezeroSteps;

    int transposeCalls = 0;

    

    public:
    
    transpose(const poseState& _nominalState ,const units::vec3 _nominalPosition , int _rezeroSteps = 100);


    void transposeSelf(const Quaternion& quant , poseState poseOfIntrest  , poseState constantReference = CoordinateSystem::WORLD_BASIS );

    const units::vec3& getNominalPosition() const {
        return nominalPosition;
    }

    const units::vec3& getTransposePosition() const {
        return transposePosition;
    }

    const poseState& getNominalState() const {
        return nominalState;
    }

    const quaternionVehicle& getTransposeState() const {
        return transposeState;
    }


    

};

}