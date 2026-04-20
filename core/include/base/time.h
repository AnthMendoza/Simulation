#pragma once
#include <cstdint>
#include <concepts>

namespace core::time{
template<std::floating_point T,std::unsigned_integral I>
const I s_to_us(const T seconds){
    return static_cast<I>(seconds * static_cast<T>(1000000));
}

template<std::floating_point T,std::unsigned_integral I>
const T us_to_s(const I microseconds) {
    return static_cast<T>(microseconds) / static_cast<T>(1000000);
}

}