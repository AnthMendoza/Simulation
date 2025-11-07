#include <gtest/gtest.h>
#include "../include/core/poseRotation.h"
#include "../include/core/quaternion.h"
#include "../include/core/coordinateSystem.h"

using namespace SimCore;


TEST(PoseRotationTest,DirectionTest_Roll_Postive){
    auto startPose = CoordinateSystem::WORLD_BASIS;
    quaternionVehicle endVehicle;
    float angle = 0.5f;
    endVehicle.eulerRotation(angle,0,0);
    auto endPose = endVehicle.getPose();
    poseAngleDifference poseAng(startPose,endPose);
    float deltaTime = 0.1f;
    auto rate = poseAng.getRotationRate(deltaTime);

    EXPECT_FLOAT_EQ(rate.rollRate,angle/deltaTime);
    EXPECT_FLOAT_EQ(rate.pitchRate,0.0f);
    EXPECT_FLOAT_EQ(rate.yawRate,0.0f);
}


TEST(PoseRotationTest,DirectionTest_Roll_Negative){
    auto startPose = CoordinateSystem::WORLD_BASIS;
    quaternionVehicle endVehicle;
    float angle = -0.5f;
    endVehicle.eulerRotation(angle,0,0);
    auto endPose = endVehicle.getPose();
    poseAngleDifference poseAng(startPose,endPose);
    float deltaTime = 0.1f;
    auto rate = poseAng.getRotationRate(deltaTime);

    EXPECT_FLOAT_EQ(rate.rollRate,angle/deltaTime);
    EXPECT_FLOAT_EQ(rate.pitchRate,0.0f);
    EXPECT_FLOAT_EQ(rate.yawRate,0.0f);
}


TEST(PoseRotationTest,DirectionTest_Pitch_Positive){
    auto startPose = CoordinateSystem::WORLD_BASIS;
    quaternionVehicle endVehicle;
    float angle = 0.5f;
    endVehicle.eulerRotation(0, angle, 0);
    auto endPose = endVehicle.getPose();
    poseAngleDifference poseAng(startPose, endPose);
    float deltaTime = 0.1f;
    auto rate = poseAng.getRotationRate(deltaTime);

    EXPECT_FLOAT_EQ(rate.rollRate, 0.0f);
    EXPECT_FLOAT_EQ(rate.pitchRate, angle / deltaTime);
    EXPECT_FLOAT_EQ(rate.yawRate, 0.0f);
}


TEST(PoseRotationTest,DirectionTest_Pitch_Negative){
    auto startPose = CoordinateSystem::WORLD_BASIS;
    quaternionVehicle endVehicle;
    float angle = -0.5f;
    endVehicle.eulerRotation(0, angle, 0);
    auto endPose = endVehicle.getPose();
    poseAngleDifference poseAng(startPose, endPose);
    float deltaTime = 0.1f;
    auto rate = poseAng.getRotationRate(deltaTime);

    EXPECT_FLOAT_EQ(rate.rollRate, 0.0f);
    EXPECT_FLOAT_EQ(rate.pitchRate, angle / deltaTime);
    EXPECT_FLOAT_EQ(rate.yawRate, 0.0f);
}


TEST(PoseRotationTest,DirectionTest_Yaw_Positive){
    auto startPose = CoordinateSystem::WORLD_BASIS;
    quaternionVehicle endVehicle;
    float angle = 0.5f;
    endVehicle.eulerRotation(0, 0, angle);
    auto endPose = endVehicle.getPose();
    poseAngleDifference poseAng(startPose, endPose);
    float deltaTime = 0.1f;
    auto rate = poseAng.getRotationRate(deltaTime);

    EXPECT_FLOAT_EQ(rate.rollRate, 0.0f);
    EXPECT_FLOAT_EQ(rate.pitchRate, 0.0f);
    EXPECT_FLOAT_EQ(rate.yawRate, angle / deltaTime);
}


TEST(PoseRotationTest,DirectionTest_Yaw_Negative){
    auto startPose = CoordinateSystem::WORLD_BASIS;
    quaternionVehicle endVehicle;
    float angle = -0.5f;
    endVehicle.eulerRotation(0, 0, angle);
    auto endPose = endVehicle.getPose();
    poseAngleDifference poseAng(startPose, endPose);
    float deltaTime = 0.1f;
    auto rate = poseAng.getRotationRate(deltaTime);

    EXPECT_FLOAT_EQ(rate.rollRate, 0.0f);
    EXPECT_FLOAT_EQ(rate.pitchRate, 0.0f);
    EXPECT_FLOAT_EQ(rate.yawRate, angle / deltaTime);
}
