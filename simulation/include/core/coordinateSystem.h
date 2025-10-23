#pragma once

#include "poseState.h"

//never changing basis. A constant Refrance for all math.
namespace CoordinateSystem {
inline const SimCore::poseState WORLD_BASIS = {
    {0, 0, 1},
    {1, 0, 0},
    {0, 1, 0} 
};
}

