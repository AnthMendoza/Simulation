#ifndef BATTERY_MODELING_H
#define BATTERY_MODELING_H

/**
 * Cross-referenced with empirical BMS data (ResearchGate):
 *   https://www.researchgate.net/figure/LiPo-Voltage-SOC-state-of-charge-table-SOC-Cell-Voltage-V-2-Cells-Voltage-V-3_tbl1_341375744
 *
 * Notes:
 *   - LiPo chemistry at ~25°C.
 *   - Temperature, aging, and C-rate will shift these values.
 */

#pragma once

#include <stdint.h>
#include <cstddef>
#include <array>

struct lipo_soc_v_point {
    uint8_t  soc_pct;   //State of charge (%)
    uint16_t ocv_mv;    //Open circuit voltage (millivolts)
};

static constexpr lipo_soc_v_point lipo_soc_v_set[] = {
    {0, 3270 },
    {5, 3610 },
    {10, 3690 },
    {15, 3710 },
    {20, 3730 },
    {25, 3750 },
    {30, 3770 },
    {35, 3790 },
    {40, 3800 },
    {45, 3820 },
    {50, 3840 },
    {55, 3850 },
    {60, 3870 },
    {65, 3910 },
    {70, 3950 },
    {75, 3980 },
    {80, 4020 },
    {85, 4080 },
    {90, 4110 },
    {95, 4150 },
    {100, 4200 }
};


constexpr size_t lipo_soc_v_size = std::size(lipo_soc_v_set);

constexpr auto make_soc_array() {
    std::array<uint8_t, lipo_soc_v_size> arr{};
    for (size_t i = 0; i < lipo_soc_v_size; i++)
        arr[i] = lipo_soc_v_set[i].soc_pct;
    return arr;
}

constexpr auto make_mv_array() {
    std::array<uint16_t, lipo_soc_v_size> arr{};
    for (size_t i = 0; i < lipo_soc_v_size; i++)
        arr[i] = lipo_soc_v_set[i].ocv_mv;
    return arr;
}
// values are set in array on compile 
static constexpr auto lipo_soc_v_x = make_soc_array();
static constexpr auto lipo_soc_v_y = make_mv_array();

//linear interpolation
uint16_t soc_to_mv(uint8_t soc) {
    //clamp
    if (soc <= lipo_soc_v_x[0]) return lipo_soc_v_y[0];
    if (soc >= lipo_soc_v_x[lipo_soc_v_size - 1]) return lipo_soc_v_y[lipo_soc_v_size - 1];

    for (size_t i = 0; i < lipo_soc_v_size - 1; i++) {
        if (soc >= lipo_soc_v_x[i] && soc <= lipo_soc_v_x[i + 1]) {
            float t = (float)(soc - lipo_soc_v_x[i]) / (lipo_soc_v_x[i + 1] - lipo_soc_v_x[i]);
            uint16_t mv = (lipo_soc_v_y[i] + t * (lipo_soc_v_y[i + 1] - lipo_soc_v_y[i]));
            return mv;
        }
    }

    return 0;
}

uint8_t mv_to_soc(uint16_t mv) {
    //clamp
    if (mv <= lipo_soc_v_y[0]) return lipo_soc_v_x[0];
    if (mv >= lipo_soc_v_y[lipo_soc_v_size - 1]) return lipo_soc_v_x[lipo_soc_v_size - 1];

    for (size_t i = 0; i < lipo_soc_v_size - 1; i++) {
        if (mv >= lipo_soc_v_y[i] && mv <= lipo_soc_v_y[i + 1]) {
            float t = (float)(mv - lipo_soc_v_y[i]) / (lipo_soc_v_y[i + 1] - lipo_soc_v_y[i]);
            uint8_t soc = (lipo_soc_v_x[i] + t * (lipo_soc_v_x[i + 1] - lipo_soc_v_x[i]));
            return soc;
        }
    }

    return 0;
}




#endif