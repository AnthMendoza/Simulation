#pragma once
#include <array>
#include <fc_units.h>

namespace avionics::sensor{

struct actual_state{

    fc_units::vec3 position;

    std::array<float,4> quaternion;

    fc_units::vec3 velocity;

};


}