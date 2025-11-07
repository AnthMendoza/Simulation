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
#include "../include/utility/utility.h"


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


TEST_F(PIDControllerFixture, controlOutputDirection ) {
    state.position = {0,0,10};
    threeDState position = state.position;
    controller->setTargetPosition(position[0],position[1],position[2]);
    controlPacks::forceMoments packet = controller->updateWithoutAllocator(0.0f,state);
    float EPSILON = 0.0001;

    EXPECT_NEAR(packet.moments[0],0.0f,EPSILON);
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
    print(packet.moments,"momtents");
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
    vehicleState.eulerRotation(0,0,M_PI_2);

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
    vehicleState.eulerRotation(0,0,M_PI_2);
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
    vehicleState.eulerRotation(0,0,M_PI_2);
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
    vehicleState.eulerRotation(0,0,M_PI_2);
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
    float gravitationalAcceleration = -9.81;
    float mass = 1;
    poseState pose = CoordinateSystem::WORLD_BASIS;
    auto request = controller->aotFeedForward(accels,mass,pose);
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
    float gravitationalAcceleration = -9.81;
    float mass = 2;
    poseState pose = CoordinateSystem::WORLD_BASIS;
    auto request = controller->aotFeedForward(accels,mass,pose);
    float EPSILON = 0.0001;

    auto dirVector = pose.dirVector;

    EXPECT_NEAR( -gravitationalAcceleration * mass + accels.zAccel * mass,request.force,EPSILON);

    EXPECT_NEAR( request.vehicleState[0],dirVector[0],EPSILON);
    EXPECT_NEAR( request.vehicleState[1],dirVector[1],EPSILON);
    EXPECT_NEAR( request.vehicleState[2],dirVector[2],EPSILON);
}




TEST_F(PIDControllerFixture,yawAngleDifference){
    poseState desired = CoordinateSystem::WORLD_BASIS;
    poseState current = CoordinateSystem::WORLD_BASIS;

    quaternionVehicle quantVehicle;
    quantVehicle.setVehicleQuaternionState(current.dirVector,current.fwdVector);
    float angle = 0.32f;
    auto yawRotation = fromAxisAngle(current.dirVector,angle);
    quantVehicle.rotatePose(yawRotation);

    threeDState axis ={1,0,0};
    auto randomRotation = fromAxisAngle(axis,0.5f);
    axis ={0,1,0};
    randomRotation = fromAxisAngle(axis,0.2f) * randomRotation;

    quantVehicle.rotatePose(randomRotation);

    current = quantVehicle.getPose();
    float calculatedAngle = controller->yawAngleDifference(desired,current);

    EXPECT_NEAR(calculatedAngle,angle,0.025f);

}




TEST_F(PIDControllerFixture, AOTTest){
    accelerations accels;
    accels.xAccel = 5;
    accels.zAccel = 0.0;
    float mass = 2;
    float grav = -9.81;
    quaternionVehicle quantVehicle;
    float angle = 0.2;
    quantVehicle.eulerRotation(0, angle, 0);
    auto pose = quantVehicle.getPose();
    auto requestedState = controller->aotFeedForward(accels, mass, pose);

    
}


TEST_F(PIDControllerFixture,AOTTestIssolatedZ){
    accelerations accels;
    accels.xAccel = 5;
    accels.zAccel = 0.0;
    float mass = 2;
    float grav = -9.81;
    quaternionVehicle quantVehicle;
    float angle = 0.2;
    quantVehicle.eulerRotation(0,angle,0);
    auto pose = quantVehicle.getPose();

    auto requestedState = controller->aotFeedForward(accels,mass,pose);
    float zforce = -grav * mass;
    auto dirVector = pose.dirVector;

    resizeVectorInPlace(dirVector,requestedState.force);
    EXPECT_NEAR(dirVector[2],-grav*mass,0.001);
}


TEST_F(PIDControllerFixture,AOTTestVerticalIssolatedZ){
    accelerations accels;
    accels.xAccel = 5;
    accels.zAccel = 0.0;
    float mass = 2;
    float grav = -9.81;
    quaternionVehicle quantVehicle;
    auto pose = quantVehicle.getPose();
    auto requestedState = controller->aotFeedForward(accels,mass,pose);
    float zforce = -grav * mass;
    auto dirVector = pose.dirVector;

    resizeVectorInPlace(dirVector,requestedState.force);
    EXPECT_NEAR(dirVector[2],-grav*mass,0.001);
}
