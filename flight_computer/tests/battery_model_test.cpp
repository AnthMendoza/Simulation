#include <gtest/gtest.h>
#include <battery_modeling.h>
#include <iostream>
TEST(SocToMvTest, mvClamps100Pct) {
    EXPECT_EQ(soc_to_mv(100), lipo_soc_v_y[lipo_soc_v_size - 1]);
}

TEST(SocToMvTest, mvClamps0Pct) {
    EXPECT_EQ(soc_to_mv(0), lipo_soc_v_y[0]);
}

TEST(SocToMvTest, mvClampsAbove100Pct) {
    EXPECT_EQ(soc_to_mv(110), lipo_soc_v_y[lipo_soc_v_size - 1]);
}


TEST(SocToMvTest, socClamps100Pct) {
    EXPECT_EQ(mv_to_soc(4200), lipo_soc_v_x[lipo_soc_v_size - 1]);
}

TEST(SocToMvTest, socClamps0Pct) {
    EXPECT_EQ(mv_to_soc(0), lipo_soc_v_x[0]);
}

TEST(SocToMvTest, socClampsAbove100Pct) {
    EXPECT_EQ(mv_to_soc(3200), lipo_soc_v_x[0]);
}

