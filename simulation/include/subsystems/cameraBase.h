#pragma once
#include <base/base.h>

namespace SimCore{

class cameraBase : public SimCore::transpose{
public:
    //nominal position to a reference frame. If the refernece frame moves the camera should move in kind
    //note rotation quanternion should be passed to transposeSelf
    cameraBase(const poseState& _nominalState ,const threeDState _nominalPosition) : transpose(_nominalState,_nominalPosition){

    }

    ~cameraBase() = default;

};

}