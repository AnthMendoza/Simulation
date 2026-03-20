#pragma once
#include <array>
namespace avionics::sensor{

struct actual_state{

    std::array<float,3> position;

    std::array<float,4> quaternion;

    std::array<float,3> velocity;

};


}