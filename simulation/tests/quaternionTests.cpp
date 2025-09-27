#include <gtest/gtest.h>
#include <cmath>
#include "../include/core/quaternion.h"

using namespace SimCore;

static poseState createIdentityPose() {
    poseState pose;
    pose.dirVector = {0.0f, 0.0f, 1.0f};
    pose.fwdVector = {1.0f, 0.0f, 0.0f};
    pose.rightVector = {0.0f, 1.0f, 0.0f};
    return pose;
}


TEST(ChangeOfBasis,RandomRotation0) {
    poseState pose = createIdentityPose();
    poseState basis = createIdentityPose();
    vehicleRefranceFrame frame(pose,basis);
    frame.realignPose(pose);
    EXPECT_TRUE(arePosesEqual(pose,basis));
}

TEST(ChangeOfBasis,RandomRotation1) {
    quaternionVehicle vehiclePose;
    vehiclePose.setVehicleQuaternionState({0,1,1},{1,0,0});
    poseState pose = vehiclePose.getPose();
    poseState basis = createIdentityPose();
    vehicleRefranceFrame frame(pose,basis);
    frame.realignPose(pose);
    EXPECT_TRUE(arePosesEqual(pose,basis));
}

TEST(ChangeOfBasis,RandomRotation2) {
    quaternionVehicle vehiclePose;
    vehiclePose.setVehicleQuaternionState({0,-1,-1},{1,0,0});
    poseState pose = vehiclePose.getPose();
    poseState basis = createIdentityPose();
    vehicleRefranceFrame frame(pose,basis);
    frame.realignPose(pose);
    EXPECT_TRUE(arePosesEqual(pose,basis));
}


TEST(ChangeOfBasis,RandomRotation3) {
    quaternionVehicle vehiclePose;
    vehiclePose.setVehicleQuaternionState({0,0,1},{1,1,0});
    poseState pose = vehiclePose.getPose();
    poseState basis = createIdentityPose();
    vehicleRefranceFrame frame(pose,basis);
    frame.realignPose(pose);
    EXPECT_TRUE(arePosesEqual(pose,basis));
}