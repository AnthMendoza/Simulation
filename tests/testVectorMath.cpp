#include "../include/vectorMath.h"
#include <array>
#include <gtest/gtest.h>



TEST(SetInBounds, AssertionTrue) {
    float upper = 3;
    float lower = 1;

    float value = -1;
    setInBounds(value , lower , upper);
    EXPECT_EQ(value , lower);

    value = 5;
    setInBounds(value , lower , upper);
    EXPECT_EQ(value, upper);
    
    value = 1.5;
    setInBounds(value , lower , upper);
    EXPECT_EQ(value, value);
}


TEST(VectorCrossProduct, AssertionTrue){
    std::array<float,3> arr1 = {3,4,5}; 
    std::array<float,3> arr2 = {1,2,3}; 
    std::array<float,3> result;
    std::array<float,3> answer1 = {2,-4,2}; 
    vectorCrossProduct(arr1, arr2, result);
    
    EXPECT_EQ(result, answer1);
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
