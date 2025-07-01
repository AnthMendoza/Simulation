#ifndef LINEARINTERPOLCATION_H
#define LINEARINTERPOLCATION_H
#include <vector>
#include <stdexcept>

template<typename type>

type linearInterpolate(const std::vector<type>& xData,const std::vector<type>& yData,type xQuery) {
    if (xData.size() != yData.size() || xData.size() < 2)
        throw std::invalid_argument("Invalid lookup table size.");


    if (xQuery <= xData.front()) return yData.front();
    if (xQuery >= xData.back()) return yData.back();

    // Find the interval xQuery is in
    for (size_t i = 0; i < xData.size() - 1; ++i) {
        if (xQuery >= xData[i] && xQuery <= xData[i + 1]) {
            type x0 = xData[i];
            type x1 = xData[i + 1];
            type y0 = yData[i];
            type y1 = yData[i + 1];

            type t = (xQuery - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }

    // Fallback (should never hit this if clamped and input is valid)
    throw std::runtime_error("Interpolation failed.");
}

#endif