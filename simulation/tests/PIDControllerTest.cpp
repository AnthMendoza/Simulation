#include <gtest/gtest.h>
#include "../include/control/dronePIDControl.h"
#include "../include/control/droneControllerBase.h"
#include "../include/utility/utility.h"
#include "../include/control/stateInfo.h"
#include "../include/utility/utility.h"
#include "../include/core/coordinateSystem.h"
#include "../include/subsystems/droneDependencyInjector.h"
#include "../include/dynamics/drone.h"
#include "../include/core/quaternion.h"


using namespace SimCore;

class PIDControllerFixture : public ::testing::Test {
protected:
    std::unique_ptr<PIDDroneController> controller;
    std::string droneConfig = "../tests/testConfigs/drone_test_config.toml";
    stateInfo state;
    

    void SetUp() override {
        controller = std::make_unique<PIDDroneController>(0.01f);
        droneConfig = readFileAsString(droneConfig);
        controller->initController(droneConfig);
        state.pose = CoordinateSystem::WORLD_BASIS;
    }

    void TearDown() override {
        controller.reset();
    }
};

enum class direction {
    positive = 1,
    negative = -1
};


static bool validateDirection(float val, direction dir) {
    if (dir == direction::positive && val > 0)
        return true;
    if (dir == direction::negative && val < 0)
        return true;
    return false;
}


TEST_F(PIDControllerFixture, controlOutputDirection ) {
    state.position = {0,0,10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0],position[1],position[2]);
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f,state);
    float EPSILON = 0.0001;

    EXPECT_TRUE(validateDirection(packet.moments[0],direction::negative));
    EXPECT_NEAR(packet.moments[1],0.0f,EPSILON);
    EXPECT_NEAR(packet.moments[2],0.0f,EPSILON);
}


TEST_F(PIDControllerFixture, controlOutputDirectionXOffsetPositive ) {
    state.position = {0,10,10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0],position[1],position[2]);
    state.position = {0,0,10};
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f,state);
    float EPSILON = 0.0001;

    print(packet.moments,"moment");
    EXPECT_TRUE(validateDirection(packet.moments[0],direction::negative));
    EXPECT_NEAR(packet.moments[1],0.0f,EPSILON);
    EXPECT_NEAR(packet.moments[2],0.0f,EPSILON);
}


TEST_F(PIDControllerFixture, controlOutputDirectionXOffsetNegative ) {
    state.position = {0,-10,10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0],position[1],position[2]);
    state.position = {0,0,10};
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f,state);
    float EPSILON = 0.0001;

    EXPECT_TRUE(validateDirection(packet.moments[0],direction::positive));
    EXPECT_NEAR(packet.moments[1],0.0f,EPSILON);
    EXPECT_NEAR(packet.moments[2],0.0f,EPSILON);
}


TEST_F(PIDControllerFixture, controlOutputDirectionYOffsetPositive) {
    state.position = {10, 0, 10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0], position[1], position[2]);
    state.position = {0,0,10};
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f, state);
    float EPSILON = 0.0001f;

    EXPECT_TRUE(validateDirection(packet.moments[1], direction::positive));
    EXPECT_NEAR(packet.moments[0], 0.0f, EPSILON);
    EXPECT_NEAR(packet.moments[2], 0.0f, EPSILON);
}


TEST_F(PIDControllerFixture, controlOutputDirectionYOffsetNegative) {
    state.position = {-10, 0, 10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0], position[1], position[2]);
    state.position = {0,0,10};
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f, state);
    float EPSILON = 0.0001f;

    EXPECT_TRUE(validateDirection(packet.moments[1], direction::negative));
    EXPECT_NEAR(packet.moments[0], 0.0f, EPSILON);
    EXPECT_NEAR(packet.moments[2], 0.0f, EPSILON);
}


TEST_F(PIDControllerFixture, rotatedControlOutputDirectionXOffsetPositive ) {
    state.position = {0,10,10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0],position[1],position[2]);
    quaternionVehicle vehicleState;
    vehicleState.eularRotation(0,0,M_PI_2);

    state.pose = vehicleState.getPose();
    state.position = {0,0,10};
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f,state);
    packet = controller->updateWithoutAllocator(0.01f,state);
    float EPSILON = 0.0001;
    print(packet.moments,"moments");

    EXPECT_TRUE(validateDirection(packet.moments[1],direction::positive));
    EXPECT_NEAR(packet.moments[0],0.0f,EPSILON);
    EXPECT_NEAR(packet.moments[2],0.0f,EPSILON);
}


TEST_F(PIDControllerFixture, rotatedControlOutputDirectionXOffsetNegative ) {
    state.position = {0,-10,10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0],position[1],position[2]);
    quaternionVehicle vehicleState;
    vehicleState.eularRotation(0,0,M_PI_2);
    state.pose = vehicleState.getPose();
    state.position = {0,0,10};
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f,state);
    float EPSILON = 0.0001;

    EXPECT_TRUE(validateDirection(packet.moments[1],direction::negative));
    EXPECT_NEAR(packet.moments[0],0.0f,EPSILON);
    EXPECT_NEAR(packet.moments[2],0.0f,EPSILON);
}


TEST_F(PIDControllerFixture, rotatedControlOutputDirectionYOffsetPositive ) {
    state.position = {10,0,10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0],position[1],position[2]);
    quaternionVehicle vehicleState;
    vehicleState.eularRotation(0,0,M_PI_2);
    state.pose = vehicleState.getPose();
    state.position = {0,0,10};
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f,state);
    float EPSILON = 0.0001;

    EXPECT_TRUE(validateDirection(packet.moments[0],direction::positive));
    EXPECT_NEAR(packet.moments[1],0.0f,EPSILON);
    EXPECT_NEAR(packet.moments[2],0.0f,EPSILON);
}


TEST_F(PIDControllerFixture, rotatedControlOutputDirectionYOffsetNegative ) {
    state.position = {-10,0,10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0],position[1],position[2]);
    quaternionVehicle vehicleState;
    vehicleState.eularRotation(0,0,M_PI_2);
    state.pose = vehicleState.getPose();
    state.position = {0,0,10};
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.01f,state);
    float EPSILON = 0.0001;
    
    EXPECT_TRUE(validateDirection(packet.moments[0],direction::negative));
    EXPECT_NEAR(packet.moments[1],0.0f,EPSILON);
    EXPECT_NEAR(packet.moments[2],0.0f,EPSILON);
}


TEST_F(PIDControllerFixture, verticalForceFeedForward){
    accelerations accels;
    float gravitationalAcceleration = -9.8;
    float mass = 1;
    poseState pose = CoordinateSystem::WORLD_BASIS;
    auto request = controller->aotFeedForward(accels,gravitationalAcceleration,mass,pose);
    float EPSILON = 0.0001;

    auto dirVector = pose.dirVector;

    EXPECT_NEAR( -gravitationalAcceleration * mass,request.force,EPSILON);

    EXPECT_NEAR( request.vehicleState[0],dirVector[0],EPSILON);
    EXPECT_NEAR( request.vehicleState[1],dirVector[1],EPSILON);
    EXPECT_NEAR( request.vehicleState[2],dirVector[2],EPSILON);
}


TEST_F(PIDControllerFixture, verticalForceFeedForwardWithZaccel){
    accelerations accels;
    accels.zAccel = 5.0f;
    float gravitationalAcceleration = -9.8;
    float mass = 2;
    poseState pose = CoordinateSystem::WORLD_BASIS;
    auto request = controller->aotFeedForward(accels,gravitationalAcceleration,mass,pose);
    float EPSILON = 0.0001;

    auto dirVector = pose.dirVector;

    EXPECT_NEAR( -gravitationalAcceleration * mass + accels.zAccel * mass,request.force,EPSILON);

    EXPECT_NEAR( request.vehicleState[0],dirVector[0],EPSILON);
    EXPECT_NEAR( request.vehicleState[1],dirVector[1],EPSILON);
    EXPECT_NEAR( request.vehicleState[2],dirVector[2],EPSILON);
}


