#pragma once

#include <cstdint>

namespace image_processing{

#pragma pack(push, 1)

struct color{
    std::uint8_t B = 0;
    std::uint8_t G = 0;
    std::uint8_t R = 0;
};

#pragma pack(pop)

}
