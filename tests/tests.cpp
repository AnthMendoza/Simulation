#include "../include/forceApplied.h"
#include "../include/vehicle.h"
#include "../include/vectorMath.h"
#include "../include/aero.h"
#include "../include/RungeKutta.h"
#include "../include/odeIterator.h"
#include "../include/rotationMatrix.h"
#include "../include/getRotation.h"
#include "../include/control.h"
#include "../include/sensors.h"
#include "../include/rocket.h"
#include "../include/toml.h"
#include <fstream>
#include <sstream>
#include <array>
#include <gtest/gtest.h>
using namespace SimCore;

//freefall, warning may fail if timestep is large as error will increase
// looking for freefall plus or minus 2 percent of actual
//this also may fail do to lack of a sample rocket configuration
TEST(System , FreeFall){
    std::ifstream inFile("../configs/Rocket_Config.toml");
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    std::string config = buffer.str();
    Rocket vehicle(config);
    vehicle.init();
    float freeFallDuration = 10000;
    vehicle.setVelocity(0.0f,0.0f,0.0f);
    vehicle.setPosition(0.0f,0.0f,freeFallDuration);
    while(vehicle.getPositionVector()[2] > 0){
        vehicle.updateState();
        ++vehicle;
    }
    float time = vehicle.getTime();
    if(time < 45.16 *1.01 && time > 45.16 * .99 ) time = 45.16;
    EXPECT_FLOAT_EQ(time , 45.16f);
}

TEST(TOML,GetValue){
    std::string configText = R"(
    [vehicle]
    dryMass = 100.0
    MOI = [1.0, 2.0, 3.0]
    gimbalDamping = 0.5
    )";

    toml::tomlParse parser;
    parser.parseConfig(configText, "vehicle");
    auto arr =parser.arrayValues["MOI"];
    EXPECT_FLOAT_EQ(arr[1], 2.0f);

}

TEST(TOML,GetValueSecondSet){
    std::string configText = R"(
    [vehicle]
    dryMass = 100.0
    MOI = [1.0, 2.0, 3.0]
    gimbalDamping = 0.5
    [rocket]
    thrust = 10000.0
    )";

    toml::tomlParse parser;
    parser.parseConfig(configText, "rocket");
    auto val =parser.floatValues["thrust"];
    EXPECT_FLOAT_EQ( val, 10000.0f);

}

TEST(TOML,BooleanTestTrue){
    std::string configText = R"(
    [vehicle]
    dryMass = 100.0
    MOI = [1.0, 2.0, 3.0]
    gimbalDamping = 0.5
    [rocket]
    thrust = 10000.0
    SetLanding = true
    )";

    toml::tomlParse parser;
    parser.parseConfig(configText, "rocket");
    auto val =parser.boolValues["SetLanding"];
    EXPECT_TRUE(val);

}

TEST(TOML,BooleanTestFalse){
    std::string configText = R"(
    [vehicle]
    dryMass = 100.0
    MOI = [1.0, 2.0, 3.0]
    gimbalDamping = 0.5
    [rocket]
    thrust = 10000.0
    SetLanding = false
    )";

    toml::tomlParse parser;
    parser.parseConfig(configText, "rocket");
    auto val =parser.boolValues["SetLanding"];
    EXPECT_FALSE(val);

}


TEST(VectorMathTest, TestVectorMagnitude) {
    std::array<float, 3> vec = {3.0f, 4.0f, 0.0f};
    EXPECT_FLOAT_EQ(vectorMag(vec), 5.0f); 
}


TEST(VectorMathTest, TestZeroVectorMagnitude) {
    std::array<float, 3> zeroVec = {0.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(vectorMag(zeroVec), 0.0f);
}


TEST(VectorMathTest, TestUnitVectorMagnitude) {
    std::array<float, 3> unitVec = {1.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(vectorMag(unitVec), 1.0f);
}


TEST(VectorMathTest, TestVectorDotProduct) {
    std::array<float, 3> vec1 = {1.0f, 2.0f, 3.0f};
    std::array<float, 3> vec2 = {4.0f, -5.0f, 6.0f};
    EXPECT_FLOAT_EQ(vectorDotProduct(vec1, vec2), 12.0f);  
}


// Test for orthogonal vectors (dot product should be 0)
TEST(VectorMathTest, TestOrthogonalDotProduct) {
    std::array<float, 3> vec1 = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> vec2 = {0.0f, 1.0f, 0.0f};
    EXPECT_FLOAT_EQ(vectorDotProduct(vec1, vec2), 0.0f);
}

// Test for dot product with zero vector (should be 0)
TEST(VectorMathTest, TestDotProductWithZeroVector) {
    std::array<float, 3> vec1 = {1.0f, 2.0f, 3.0f};
    std::array<float, 3> zeroVec = {0.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(vectorDotProduct(vec1, zeroVec), 0.0f);
}

// Test for dot product with the same vector (should be the square of the vector's magnitude)
TEST(VectorMathTest, TestDotProductWithSameVector) {
    std::array<float, 3> vec1 = {1.0f, 2.0f, 3.0f};
    EXPECT_FLOAT_EQ(vectorDotProduct(vec1, vec1), 14.0f);  // (1^2 + 2^2 + 3^2)
}




TEST(VectorMathTest, TestVectorCrossProduct) {
    std::array<float, 3> vec1 = {1.0f, 2.0f, 3.0f};
    std::array<float, 3> vec2 = {4.0f, 5.0f, 6.0f};
    std::array<float, 3> result;
    vectorCrossProduct(vec1, vec2, result);
    std::array<float, 3> expected = {-3.0f, 6.0f, -3.0f};
    EXPECT_EQ(result, expected); 
}

// Test for parallel vectors (cross product should be zero vector)
TEST(VectorMathTest, TestParallelCrossProduct) {
    std::array<float, 3> vec1 = {1.0f, 2.0f, 3.0f};
    std::array<float, 3> vec2 = {2.0f, 4.0f, 6.0f};  // 2*vec1
    std::array<float, 3> result;
    vectorCrossProduct(vec1, vec2, result);
    std::array<float, 3> expected = {0.0f, 0.0f, 0.0f};
    EXPECT_EQ(result, expected);
}

// Test for perpendicular vectors (cross product should give non-zero result)
TEST(VectorMathTest, TestPerpendicularCrossProduct) {
    std::array<float, 3> vec1 = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> vec2 = {0.0f, 1.0f, 0.0f};
    std::array<float, 3> result;
    vectorCrossProduct(vec1, vec2, result);
    std::array<float, 3> expected = {0.0f, 0.0f, 1.0f};
    EXPECT_EQ(result, expected);
}


TEST(VectorMathTest, TestVectorAngleBetween) {
    std::array<float, 3> vec1 = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> vec2 = {0.0f, 1.0f, 0.0f};
    EXPECT_FLOAT_EQ(vectorAngleBetween(vec1, vec2), M_PI / 2);  
}

// Test for angle between two identical vectors (should be 0 radians)
TEST(VectorMathTest, TestAngleBetweenSameVectors) {
    std::array<float, 3> vec1 = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> vec2 = {1.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(vectorAngleBetween(vec1, vec2), 0.0f);
}

// Test for angle between two opposite vectors (should be π radians)
TEST(VectorMathTest, TestAngleBetweenOppositeVectors) {
    std::array<float, 3> vec1 = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> vec2 = {-1.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(vectorAngleBetween(vec1, vec2), M_PI);
}



TEST(VectorMathTest, TestNormalizeVector) {
    std::array<float, 3> vec = {3.0f, 4.0f, 0.0f};
    std::array<float, 3> normalized = normalizeVector(vec);
    std::array<float, 3> expected = {0.6f, 0.8f, 0.0f};  
    
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(normalized[i], expected[i], 0.001f); 
    }
}

// Test for very small vector (should still work)
TEST(VectorMathTest, TestVerySmallVectorMagnitude) {
    std::array<float, 3> smallVec = {1e-6f, 1e-6f, 1e-6f};
    EXPECT_FLOAT_EQ(vectorMag(smallVec), sqrt(3) * 1e-6f);
}

// Test for basic vector addition
TEST(VectorMathTest, TestVectorAddition) {
    std::array<float, 3> vec1 = {1.0f, 2.0f, 3.0f};
    std::array<float, 3> vec2 = {4.0f, 5.0f, 6.0f};
    
    std::array<float, 3> result = addVectors(vec1, vec2);
    std::array<float, 3> expected = {5.0f, 7.0f, 9.0f};
    
    EXPECT_EQ(result, expected);
}

// Test for adding a zero vector
TEST(VectorMathTest, TestVectorAdditionWithZero) {
    std::array<float, 3> vec1 = {1.0f, 2.0f, 3.0f};
    std::array<float, 3> zeroVec = {0.0f, 0.0f, 0.0f};
    
    std::array<float, 3> result = addVectors(vec1, zeroVec);
    EXPECT_EQ(result, vec1);
}

// Test for adding vectors with negative values
TEST(VectorMathTest, TestVectorAdditionWithNegativeValues) {
    std::array<float, 3> vec1 = {-1.0f, -2.0f, -3.0f};
    std::array<float, 3> vec2 = {4.0f, 5.0f, 6.0f};
    
    std::array<float, 3> result = addVectors(vec1, vec2);
    std::array<float, 3> expected = {3.0f, 3.0f, 3.0f};
    
    EXPECT_EQ(result, expected);
}

// Test for adding vectors with mixed signs
TEST(VectorMathTest, TestVectorAdditionWithMixedSigns) {
    std::array<float, 3> vec1 = {1.0f, -2.0f, 3.0f};
    std::array<float, 3> vec2 = {-1.0f, 2.0f, -3.0f};
    
    std::array<float, 3> result = addVectors(vec1, vec2);
    std::array<float, 3> expected = {0.0f, 0.0f, 0.0f};
    
    EXPECT_EQ(result, expected);
}

// Test for adding vectors with large values
TEST(VectorMathTest, TestVectorAdditionWithLargeValues) {
    std::array<float, 3> vec1 = {1e6f, 1e6f, 1e6f};
    std::array<float, 3> vec2 = {1e6f, 1e6f, 1e6f};
    
    std::array<float, 3> result = addVectors(vec1, vec2);
    std::array<float, 3> expected = {2e6f, 2e6f, 2e6f};
    
    EXPECT_EQ(result, expected);
}

// Test for adding vectors with zero values
TEST(VectorMathTest, TestVectorAdditionWithZeroValues) {
    std::array<float, 3> vec1 = {0.0f, 2.0f, 0.0f};
    std::array<float, 3> vec2 = {1.0f, 0.0f, 3.0f};
    
    std::array<float, 3> result = addVectors(vec1, vec2);
    std::array<float, 3> expected = {1.0f, 2.0f, 3.0f};
    
    EXPECT_EQ(result, expected);
}

// Test case for matrix multiplication
TEST(MatrixTest, MatrixMultiplication) {
    Matrix3x3 mat1;
    mat1.m = {{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}}};

    Matrix3x3 mat2;
    mat2.m = {{{9.0f, 8.0f, 7.0f}, {6.0f, 5.0f, 4.0f}, {3.0f, 2.0f, 1.0f}}};

    Matrix3x3 result = mat1 * mat2;

    std::array<std::array<float, 3>, 3> expected = {{{30.0f, 24.0f, 18.0f}, {84.0f, 69.0f, 54.0f}, {138.0f, 114.0f, 90.0f}}};

    EXPECT_EQ(result.m, expected);
}


TEST(MatrixTest, RotationMatrixX) {
    float angle = M_PI / 2; // 90 degrees in radians
    Matrix3x3 rot = rotationMatrixX(angle);
    
    std::array<float, 3> vec = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> rotatedVec = rot.rotate(vec);
    
    // Expected result after 90 degree rotation around X-axis
    std::array<float, 3> expected = {1.0f, 0.0f, 0.0f};
    
    EXPECT_EQ(rotatedVec, expected);
}


TEST(MatrixTest, RotationMatrixY) {
    float angle = M_PI / 2; // 90 degrees in radians
    Matrix3x3 rot = rotationMatrixY(angle);
    
    std::array<float, 3> vec = {0.0f, 1.0f, 0.0f};
    std::array<float, 3> rotatedVec = rot.rotate(vec);
    
    // Expected result after 90 degree rotation around Y-axis
    std::array<float, 3> expected = {0.0f, 1.0f, 0.0f};
    
    EXPECT_EQ(rotatedVec, expected);
}

// Test case for rotating a zero vector
TEST(MatrixTest, RotateZeroVector) {
    Matrix3x3 rot = rotationMatrixZ(M_PI / 2); // 90 degrees
    
    std::array<float, 3> zeroVec = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> rotatedVec = rot.rotate(zeroVec);
    
    // Zero vector should remain zero after any rotation
    std::array<float, 3> expected = {0.0f, 0.0f, 0.0f};
    
    EXPECT_EQ(rotatedVec, expected);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
