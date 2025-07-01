#include "../include/core/forceApplied.h"
#include "../include/dynamics/vehicle.h"
#include "../include/core/vectorMath.h"
#include "../include/dynamics/aero.h"
#include "../include/core/RungeKutta.h"
#include "../include/core/odeIterator.h"
#include "../include/core/rotationMatrix.h"
#include "../include/core/getRotation.h"
#include "../include/control/control.h"
#include "../include/subsystems/sensors.h"
#include "../include/dynamics/rocket.h"
#include "../include/sim/toml.h"
#include "../include/core/quaternion.h"
#include "../include/core/linearInterpolation.h"
#include "../include/control/droneControl.h"
#include "../include/dynamics/drone.h"
#include "../include/thirdparty/Eigen/Eigen"
#include "../include/thirdparty/Eigen/Dense"
#include <fstream>
#include <sstream>
#include <array>
#include <gtest/gtest.h>
using namespace SimCore;

constexpr float EPSILON = 1e-6;
//freefall, warning may fail if timestep is large as error will increase
// looking for freefall plus or minus 2 percent of actual
//this also may fail do to lack of a sample rocket configuration
TEST(System , FreeFall){
    std::ifstream inFile("../configs/Rocket_Config.toml");
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    std::string config = buffer.str();
    Rocket vehicle(config);
    vehicle.init(config);
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


TEST(ControlAllocator, WrenchMatchesDesiredInput) {
    std::vector<std::array<float, 3>> positions = {
        {{1, 1, 0}}, {{-1, 1, 0}}, {{-1, -1, 0}}, {{1, -1, 0}}
    };
    std::vector<std::array<float, 3>> thrustDirs = {
        {{0, 0, 1}}, {{0, 0, 1}}, {{0, 0, 1}}, {{0, 0, 1}}
    };
    std::vector<float> spin = {1, -1, 1, -1};

    controlAllocator allocator(positions, thrustDirs, spin);

    VectorXd desired = allocator.toVectorXd({0, 0, 4, 0, 0, 0});
    VectorXd thrusts = allocator.allocate(desired);
    VectorXd actual = allocator.computeWrench(thrusts);
    for(int i = 0; i < 6; ++i) {
        EXPECT_NEAR(actual(i), desired(i), EPSILON);
    }
}



TEST(LinearInterpolationTest, InterpolatesCorrectlyInMiddle) {
    std::vector<float> x = {0.0f, 1.0f, 2.0f};
    std::vector<float> y = {0.0f, 10.0f, 20.0f};

    float result = linearInterpolate(x, y, 1.5f);
    EXPECT_NEAR(result, 15.0f, EPSILON);
}


TEST(LinearInterpolationTest, ClampsToLowerBound) {
    std::vector<float> x = {0.0f, 1.0f};
    std::vector<float> y = {10.0f, 20.0f};

    float result = linearInterpolate(x, y, -1.0f);
    EXPECT_NEAR(result, 10.0f, EPSILON);
}

TEST(LinearInterpolationTest, ClampsToUpperBound) {
    std::vector<float> x = {0.0f, 1.0f};
    std::vector<float> y = {10.0f, 20.0f};

    float result = linearInterpolate(x, y, 2.0f);
    EXPECT_NEAR(result, 20.0f, EPSILON);
}


TEST(LinearInterpolationTest, ReturnsExactMatch) {
    std::vector<float> x = {0.0f, 1.0f, 2.0f};
    std::vector<float> y = {5.0f, 15.0f, 25.0f};

    float result = linearInterpolate(x, y, 1.0f);
    EXPECT_NEAR(result, 15.0f, EPSILON);
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
    auto arr =parser.getArray("MOI");
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
    auto val =parser.getFloat("thrust");
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
    auto val =parser.getBool("SetLanding");
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
    auto val =parser.getBool("SetLanding");
    EXPECT_FALSE(val);

}




bool almostEqual(float a, float b, float eps = EPSILON) {
    return std::abs(a - b) < eps;
}

bool vectorsAlmostEqual(const std::array<float, 3>& v1, const std::array<float, 3>& v2, float eps = EPSILON) {
    return almostEqual(v1[0], v2[0], eps) &&
           almostEqual(v1[1], v2[1], eps) &&
           almostEqual(v1[2], v2[2], eps);
}

TEST(QuaternionRotationTest, IdentityRotation) {
    SimCore::Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};  // Identity quaternion
    std::array<float, 3> v = {1.0f, 2.0f, 3.0f};

    auto rotated = rotateVector(q, v);

    EXPECT_TRUE(vectorsAlmostEqual(rotated, v));
}
// 90° rotation around Z-axis
TEST(QuaternionRotationTest, Rotate90DegreesAroundZ) {
    float angle_rad = M_PI / 2.0f;
    std::array<float, 3> axis = {0.0f, 0.0f, 1.0f};
    SimCore::Quaternion q = fromAxisAngle(axis, angle_rad);

    std::array<float, 3> v = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> expected = {0.0f, 1.0f, 0.0f};

    auto rotated = rotateVector(q, v);

    EXPECT_TRUE(vectorsAlmostEqual(rotated, expected));
}
// 180° rotation around Y-axis
TEST(QuaternionRotationTest, Rotate180DegreesAroundY) {
    float angle_rad = M_PI;
    std::array<float, 3> axis = {0.0f, 1.0f, 0.0f};
    SimCore::Quaternion q = fromAxisAngle(axis, angle_rad);

    std::array<float, 3> v = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> expected = {-1.0f, 0.0f, 0.0f};

    auto rotated = rotateVector(q, v);

    EXPECT_TRUE(vectorsAlmostEqual(rotated, expected));
}

TEST(QuaternionRotationTest, Rotate45DegreesAroundX) {
    float angle_rad = M_PI / 4.0f;
    std::array<float, 3> axis = {1.0f, 0.0f, 0.0f};
    SimCore::Quaternion q = fromAxisAngle(axis, angle_rad);

    std::array<float, 3> v = {0.0f, 1.0f, 0.0f};
    std::array<float, 3> expected = {
        0.0f,
        std::cos(angle_rad),
        std::sin(angle_rad)
    };

    auto rotated = rotateVector(q, v);

    EXPECT_TRUE(almostEqual(rotated[0], expected[0]));
    EXPECT_TRUE(almostEqual(rotated[1], expected[1]));
    EXPECT_TRUE(almostEqual(rotated[2], expected[2]));
}

TEST(QuaternionRotationTest, CombinedRotationTest) {
    // Step 1: Define two 90° rotations
    float angle_rad = M_PI / 2.0f;

    SimCore::Quaternion q_z = fromAxisAngle({0.0f, 0.0f, 1.0f}, angle_rad);  // Rotate around Z
    SimCore::Quaternion q_y = fromAxisAngle({0.0f, 1.0f, 0.0f}, angle_rad);  // Rotate around Y

    SimCore::Quaternion q_combined = q_y * q_z;
    
    std::array<float, 3> v = {1.0f, 0.0f, 0.0f};
    std::array<float, 3> expected = {0.0f, 1.0f, 0.0f};
    auto rotated = rotateVector(q_combined, v);
    EXPECT_TRUE(vectorsAlmostEqual(rotated, expected));
}


bool almostEqual(const std::array<float, 3>& a, const std::array<float, 3>& b, float epsilon = EPSILON) {
    return std::fabs(a[0] - b[0]) < epsilon &&
           std::fabs(a[1] - b[1]) < epsilon &&
           std::fabs(a[2] - b[2]) < epsilon;
}

TEST(QuaternionVehicleTest, Rotate90DegreesAroundX) {
    quaternionVehicle vehicle({0,1,0} , {0,0,1});
    vehicle.eularRotation(M_PI / 2, 0, 0);  // 90 degrees around X

    std::array<float, 3> expectedDir = {0, 0, 1};
    std::array<float, 3> expectedFwd = {0, -1, 0};

    EXPECT_TRUE(almostEqual(vehicle.getdirVector(), expectedDir));
    EXPECT_TRUE(almostEqual(vehicle.getfwdVector(), expectedFwd));
}

TEST(QuaternionVehicleTest, Rotate90DegreesAroundY) {
    quaternionVehicle vehicle({1,0,0},{0,0,1});

    vehicle.eularRotation(0, M_PI / 2, 0);  // 90 degrees around Y

    std::array<float, 3> expectedDir = {0, 0, -1};
    std::array<float, 3> expectedFwd = {1, 0, 0};
    EXPECT_TRUE(almostEqual(vehicle.getdirVector(), expectedDir));
    EXPECT_TRUE(almostEqual(vehicle.getfwdVector(), expectedFwd));
}

TEST(QuaternionVehicleTest, Rotate90DegreesAroundZ) {
    quaternionVehicle vehicle({1, 0, 0},{0, 1, 0});

    vehicle.eularRotation(0, 0, M_PI / 2);  // 90 degrees around Z

    std::array<float, 3> expectedDir = {-1 * 0, 1, 0};  // same Y
    std::array<float, 3> expectedFwd = {-1, 0, 0};      // rotated from X

    EXPECT_TRUE(almostEqual(vehicle.getdirVector(), expectedDir));
    EXPECT_TRUE(almostEqual(vehicle.getfwdVector(), expectedFwd));
}

TEST(QuaternionVehicleTest, CombinedRotationXYZ) {
    quaternionVehicle vehicle({0, 1, 0}, {1, 0, 0});

    vehicle.eularRotation(M_PI / 2, M_PI / 2, 0);  // Only once!

    // Build combined rotation manually
    SimCore::Quaternion qx = fromAxisAngle({1, 0, 0}, M_PI / 2);
    SimCore::Quaternion qy = fromAxisAngle({0, 1, 0}, M_PI / 2);
    SimCore::Quaternion combined = qy * qx;  // Rotate X first, then Y

    std::array<float, 3> expectedDir = rotateVector(combined, {0, 1, 0});
    std::array<float, 3> expectedFwd = rotateVector(combined, {1, 0, 0});

    EXPECT_TRUE(almostEqual(vehicle.getdirVector(), expectedDir));
    EXPECT_TRUE(almostEqual(vehicle.getfwdVector(), expectedFwd));
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


// Test for zeroVector True
TEST(VectorMathTest, zeroVectorTrue) {
    std::array<float, 3> vec = {0.0f, 0.0f, 0.0f};
    EXPECT_TRUE(isZeroVector(vec));
}


// Test for zeroVector False
TEST(VectorMathTest, zeroVectorFalse) {
    std::array<float, 3> vec = {1.0f, 0.0f, 0.0f};
    EXPECT_FALSE(isZeroVector(vec));
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


TEST(EllipsoidalClamp2DTest, InsideEllipse_NoClamp) {
    auto [x, y] = ellipsoidalClamp2D(2.0f, 1.0f, 4.0f, 2.0f);
    EXPECT_TRUE(almostEqual(x, 2.0f,EPSILON));
    EXPECT_TRUE(almostEqual(y, 1.0f,EPSILON));
}

TEST(EllipsoidalClamp2DTest, OutsideEllipse_ClampedToBoundary) {
    auto [x, y] = ellipsoidalClamp2D(8.0f, 4.0f, 4.0f, 2.0f);
    float normalized = (x * x) / (4.0f * 4.0f) + (y * y) / (2.0f * 2.0f);
    EXPECT_TRUE(almostEqual(normalized, 1.0f,EPSILON));
}

TEST(EllipsoidalClamp2DTest, DirectionPreservedAfterClamp) {
    float origX = 10.0f, origY = 0.0f;
    auto [x, y] = ellipsoidalClamp2D(origX, origY, 4.0f, 2.0f);
    EXPECT_TRUE(x > 0 && y == 0.0f);
    EXPECT_TRUE(almostEqual(x, 4.0f,EPSILON));
}

TEST(EllipsoidalClamp3DTest, InsideEllipsoid_NoClamp) {
    auto [x, y, z] = ellipsoidalClamp(1.0f, 1.0f, 1.0f, 4.0f, 5.0f, 6.0f);
    EXPECT_TRUE(almostEqual(x, 1.0f , EPSILON));
    EXPECT_TRUE(almostEqual(y, 1.0f , EPSILON));
    EXPECT_TRUE(almostEqual(z, 1.0f , EPSILON));
}

TEST(EllipsoidalClamp3DTest, OutsideEllipsoid_ClampedToBoundary) {
    auto [x, y, z] = ellipsoidalClamp(10.0f, 10.0f, 10.0f, 4.0f, 5.0f, 6.0f);
    float value = (x * x) / (4.0f * 4.0f) + (y * y) / (5.0f * 5.0f) + (z * z) / (6.0f * 6.0f);
    EXPECT_TRUE(almostEqual(value, 1.0f,EPSILON));
}

TEST(EllipsoidalClamp3DTest, DirectionPreservedAfterClamp) {
    float origX = 10.0f, origY = 0.0f, origZ = 0.0f;
    auto [x, y, z] = ellipsoidalClamp(origX, origY, origZ, 4.0f, 5.0f, 6.0f);
    EXPECT_TRUE(x > 0 && y == 0.0f && z == 0.0f);
    EXPECT_TRUE(almostEqual(x, 4.0f,EPSILON));
}





int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
