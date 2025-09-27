#include <gtest/gtest.h>
#include <cmath>
#include "../include/core/poseRotation.h" 
#include "../include/core/quaternion.h"

using namespace SimCore;

static poseState createIdentityPose() {
    poseState pose;
    pose.rightVector = {1.0f, 0.0f, 0.0f};
    pose.dirVector = {0.0f, 0.0f, 1.0f};
    pose.fwdVector = {0.0f, 1.0f, 0.0f};
    return pose;
}

bool floatEqual(float a, float b, float tolerance = 1e-4f) {
    return std::abs(a - b) < tolerance;
}

TEST(PoseAngleDifferenceTest, IdenticalPoses) {
    poseState start = createIdentityPose();
    poseState end = createIdentityPose();
    
    SimCore::poseAngleDifference calc(start, end);
    rotations result = calc.getDifference();
    
    EXPECT_TRUE(floatEqual(result.roll, 0.0f));
    EXPECT_TRUE(floatEqual(result.pitch, 0.0f));
    EXPECT_TRUE(floatEqual(result.yaw, 0.0f));
}

TEST(PoseAngleDifferenceTest, PureRollPositive) {
    poseState start = createIdentityPose();
    poseState end = createIdentityPose();
    
    float angle = M_PI / 2;
    end.rightVector = {0.0f, 0.0f, 1.0f};
    end.dirVector = {-1.0f, 0.0f, 0.0f};
    
    SimCore::poseAngleDifference calc(start, end);
    rotations result = calc.getDifference();
    
    EXPECT_TRUE(floatEqual(result.roll, angle));
    EXPECT_TRUE(floatEqual(result.pitch, 0.0f));
    EXPECT_TRUE(floatEqual(result.yaw, 0.0f));
}

TEST(PoseAngleDifferenceTest, PureRollNegative) {
    poseState start = createIdentityPose();
    poseState end = createIdentityPose();
    
    float angle = -M_PI / 2;
    end.rightVector = {0.0f, 0.0f, -1.0f};
    end.dirVector = {1.0f, 0.0f, 0.0f};
    
    SimCore::poseAngleDifference calc(start, end);
    rotations result = calc.getDifference();
    
    EXPECT_TRUE(floatEqual(result.roll, angle));
    EXPECT_TRUE(floatEqual(result.pitch, 0.0f));
    EXPECT_TRUE(floatEqual(result.yaw, 0.0f));
}

TEST(PoseAngleDifferenceTest, PurePitchPositive) {
    poseState start = createIdentityPose();
    poseState end = createIdentityPose();
    
    float angle = M_PI / 2;
    end.fwdVector = {0.0f, 0.0f, 1.0f};
    end.dirVector = {0.0f, -1.0f, 0.0f};
    
    SimCore::poseAngleDifference calc(start, end);
    rotations result = calc.getDifference();
    
    EXPECT_TRUE(floatEqual(result.roll, 0.0f));
    EXPECT_TRUE(floatEqual(result.pitch, angle));
    EXPECT_TRUE(floatEqual(result.yaw, 0.0f));
}

TEST(PoseAngleDifferenceTest, PureYawPositive) {
    poseState start = createIdentityPose();
    poseState end = createIdentityPose();
    
    float angle = M_PI / 2;
    end.fwdVector = {1.0f, 0.0f, 0.0f};
    end.rightVector = {0.0f, -1.0f, 0.0f};
    
    SimCore::poseAngleDifference calc(start, end);
    rotations result = calc.getDifference();
    
    EXPECT_TRUE(floatEqual(result.roll, 0.0f));
    EXPECT_TRUE(floatEqual(result.pitch, 0.0f));
    EXPECT_TRUE(floatEqual(result.yaw, angle));
}

TEST(PoseAngleDifferenceTest, SmallAngles) {
    poseState start = createIdentityPose();
    poseState end = createIdentityPose();
    
    float angle = M_PI / 18;
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    
    end.rightVector = {cos_a, 0.0f, sin_a};
    end.dirVector = {-sin_a, 0.0f, cos_a};
    
    SimCore::poseAngleDifference calc(start, end);
    rotations result = calc.getDifference();
    
    EXPECT_TRUE(floatEqual(result.roll, angle));
    EXPECT_TRUE(floatEqual(result.pitch, 0.0f));
    EXPECT_TRUE(floatEqual(result.yaw, 0.0f));
}

TEST(PoseAngleDifferenceTest, SettersWork) {
    poseState start = createIdentityPose();
    poseState end = createIdentityPose();
    poseState newEnd = createIdentityPose();
    
    float angle = M_PI / 4;
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    
    newEnd.fwdVector = {sin_a, cos_a, 0.0f};
    newEnd.rightVector = {cos_a, -sin_a, 0.0f};
    
    SimCore::poseAngleDifference calc(start, end);
    calc.setEndPose(newEnd);
    
    rotations result = calc.getDifference();
    
    EXPECT_TRUE(floatEqual(result.yaw, angle));
    EXPECT_TRUE(floatEqual(result.roll, 0.0f));
    EXPECT_TRUE(floatEqual(result.pitch, 0.0f));
}

TEST(PoseAngleDifferenceTest, LargeAngles) {
    poseState start = createIdentityPose();
    poseState end = createIdentityPose();
    
    end.fwdVector = {0.0f, -1.0f, 0.0f};
    end.rightVector = {-1.0f, 0.0f, 0.0f};
    
    SimCore::poseAngleDifference calc(start, end);
    rotations result = calc.getDifference();
    
    EXPECT_TRUE(floatEqual(std::abs(result.yaw), M_PI));
    EXPECT_TRUE(floatEqual(result.roll, 0.0f));
    EXPECT_TRUE(floatEqual(result.pitch, 0.0f));
}