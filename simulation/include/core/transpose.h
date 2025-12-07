#pragma once


#include "quaternion.h"
#include "vectorMath.h"
#include "coordinateSystem.h"


namespace SimCore {

class transpose {
    private:

    threeDState nominalPosition;
    threeDState transposePosition;

    poseState nominalState;
    quaternionVehicle transposeState;

    int rezeroSteps;

    int transposeCalls = 0;

    

    public:
    
    transpose(const poseState& _nominalState ,const threeDState _nominalPosition , int _rezeroSteps = 100);


    void transposeSelf(const Quaternion& quant , poseState poseOfIntrest  , poseState constantReference = CoordinateSystem::WORLD_BASIS );

    const threeDState& getNominalPosition() const {
        return nominalPosition;
    }

    const threeDState& getTransposePosition() const {
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