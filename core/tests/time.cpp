#include <base/time.h>
#include <gtest/gtest.h>


TEST(TimeConversion, SecondsToMicrosecondsBasic) {
    EXPECT_EQ((core::time::s_to_us<double, uint64_t>(1.0)), 1000000);
    EXPECT_EQ((core::time::s_to_us<double, uint64_t>(0.0)), 0);
    EXPECT_EQ((core::time::s_to_us<double, uint64_t>(0.5)), 500000);
    EXPECT_EQ((core::time::s_to_us<float, uint32_t>(2.0f)), 2000000);
}

TEST(TimeConversion, MicrosecondsToSecondsBasic) {
    EXPECT_DOUBLE_EQ((core::time::us_to_s<double, uint64_t>(1000000)), 1.0);
    EXPECT_DOUBLE_EQ((core::time::us_to_s<double, uint64_t>(0)), 0.0);
    EXPECT_DOUBLE_EQ((core::time::us_to_s<double, uint64_t>(500000)), 0.5);
}

TEST(TimeConversion, RoundTrip) {
    double input = 1.234567;
    auto us = core::time::s_to_us<double, uint64_t>(input);
    double result = core::time::us_to_s<double, uint64_t>(us);

    EXPECT_NEAR(result, input, 1e-8);
}

TEST(TimeConversion, LargeValues) {
    double input = 10000000.0; //~116 days.
    auto us = core::time::s_to_us<double, uint64_t>(input);
    uint64_t expected = 10000000000000;  
    EXPECT_EQ(us, expected );

    double back = core::time::us_to_s<double, uint64_t>(us);
    EXPECT_DOUBLE_EQ(back, input);
}