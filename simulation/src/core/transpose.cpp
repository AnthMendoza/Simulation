#include "../../include/core/transpose.h"


SimCore::transpose::transpose(const poseState& _nominalState ,const threeDState _nominalPosition , int _rezeroSteps)
    : nominalPosition(_nominalPosition) , nominalState(_nominalState)
    , transposePosition(_nominalPosition) , rezeroSteps(_rezeroSteps){

    transposeState.setVehicleQuaternionState(nominalState.dirVector,nominalState.fwdVector);

    }  


void SimCore::transpose::transposeSelf(const Quaternion& quant , poseState poseOfIntrest  , poseState constantReference){

    transposeCalls += 1;

    rotateVector(quant , transposePosition);
    transposeState.rotatePose(quant);
    
    if(transposeCalls < rezeroSteps) return;
    transposeCalls = 0;

    vehicleReferenceFrame referenceFrame(poseOfIntrest,constantReference);

    poseState reframePose = nominalState;

    referenceFrame.realignPose(reframePose);

    transposeState.setVehicleQuaternionState(reframePose.dirVector,reframePose.fwdVector);

    threeDState reframePosition = nominalPosition;

    referenceFrame.realign(reframePosition);

    transposePosition = reframePosition;
    
}