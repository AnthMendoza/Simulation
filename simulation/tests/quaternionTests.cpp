#include <gtest/gtest.h>
#include <cmath>
#include "../include/core/quaternion.h"


using namespace SimCore;


TEST(ChangeOfBasis,RandomRotation0) {
    poseState pose = CoordinateSystem::WORLD_BASIS;
    poseState basis = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame frame(pose,basis);
    frame.realignPose(pose);
    bool testResult = arePosesEqual(pose,basis);
    if(!testResult){
        pose.printPose("Pose");
        basis.printPose("Basis");
    }
    EXPECT_TRUE(testResult);
}

TEST(ChangeOfBasis,RandomRotation1) {
    quaternionVehicle vehiclePose;
    vehiclePose.setVehicleQuaternionState({0,1,1},{1,0,0});
    poseState pose = vehiclePose.getPose();
    poseState basis = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame frame(pose,basis);
    frame.realignPose(pose);
    bool testResult = arePosesEqual(pose,basis);
    if(!testResult){
        pose.printPose("Pose");
        basis.printPose("Basis");
    }
    EXPECT_TRUE(testResult);
}

TEST(ChangeOfBasis,RandomRotation2) {
    quaternionVehicle vehiclePose;
    vehiclePose.setVehicleQuaternionState({0,-1,-1},{1,0,0});
    poseState pose = vehiclePose.getPose();
    poseState basis = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame frame(pose,basis);
    frame.realignPose(pose);
    bool testResult = arePosesEqual(pose,basis);
    if(!testResult){
        pose.printPose("Pose");
        basis.printPose("Basis");
    }
    EXPECT_TRUE(testResult);
}


TEST(ChangeOfBasis,RandomRotation3) {
    quaternionVehicle vehiclePose;
    vehiclePose.setVehicleQuaternionState({0,0,1},{1,1,0});
    poseState pose = vehiclePose.getPose();
    poseState basis = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame frame(pose,basis);
    frame.realignPose(pose);
    bool testResult = arePosesEqual(pose,basis);
    if(!testResult){
        pose.printPose("Pose");
        basis.printPose("Basis");
    }
    EXPECT_TRUE(testResult);
}

TEST(ChangeOfBasis,RandomRotation4) {
    quaternionVehicle vehiclePose;
    vehiclePose.setVehicleQuaternionState({0,0,1},{0,1,0});
    poseState pose = vehiclePose.getPose();
    poseState basis = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame frame(pose,basis);
    frame.realignPose(pose);
    bool testResult = arePosesEqual(pose,basis);
    if(!testResult){
        pose.printPose("Pose");
        basis.printPose("Basis");
    }
    EXPECT_TRUE(testResult);
}
TEST(ChangeOfBasis,RandomRotation5) {
    quaternionVehicle vehiclePose;
    vehiclePose.setVehicleQuaternionState({0,0,1},{1,1,0});
    poseState pose = vehiclePose.getPose();
    poseState basis = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame frame(pose,basis);
    frame.realignPose(pose);
    bool testResult = arePosesEqual(pose,basis);
    if(!testResult){
        pose.printPose("Pose");
        basis.printPose("Basis");
    }
    EXPECT_TRUE(testResult);
}