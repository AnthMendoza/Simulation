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

class PIDDroneControllerFixture : public ::testing::Test {
protected:
   
    std::string droneConfig = "../tests/testConfigs/drone_test_config.toml";
    stateInfo state;


    class PIDDroneControllerPassThrough: public PIDDroneController {
    public:
        using PIDDroneController::PIDDroneController;


        using PIDDroneController::computeAccelerationRequest;
        using PIDDroneController::computeVehicleBasisAOT;
        using PIDDroneController::computeAngularVelocityFromAOT;
        using PIDDroneController::computeMomentFromAOT;
        using PIDDroneController::computeThrustForce;

        eulerAOT createEuler(float X, float Y){
            eulerAOT euler;
            euler.angleX = X;
            euler.angleY = Y;
            return euler;
        }
    };

    std::unique_ptr<PIDDroneControllerPassThrough> controller;

    void SetUp() override {
        controller = std::make_unique<PIDDroneControllerPassThrough>(0.01f);
        droneConfig = readFileAsString(droneConfig);
        controller->initController(droneConfig);
        state.pose = CoordinateSystem::WORLD_BASIS;
    }

    void TearDown() override {
        controller.reset();
    }
};


TEST_F(PIDDroneControllerFixture, ComputesExpectedAccelerationXPositive) {
    units::vec3 position = { 10.0f, 5.0f, 2.0f };
    units::vec3 velocity = { 0.0f, 0.0f, 0.0f };
    float deltaTime = 0.1f;
    controller->setTargetPosition(position[0] + 5, position[1], position[2]);
    auto result = controller->computeAccelerationRequest(position, velocity, deltaTime);

    EXPECT_TRUE(validateDirection(result.xAccel, direction::positive));
}

TEST_F(PIDDroneControllerFixture, ComputesExpectedAccelerationXNegative) {
    units::vec3 position = { 10.0f, 5.0f, 2.0f };
    units::vec3 velocity = { 0.0f, 0.0f, 0.0f };
    float deltaTime = 0.1f;
    controller->setTargetPosition(position[0] - 5, position[1], position[2]);
    auto result = controller->computeAccelerationRequest(position, velocity, deltaTime);

    EXPECT_TRUE(validateDirection(result.xAccel, direction::negative));
}

TEST_F(PIDDroneControllerFixture, ComputesExpectedAccelerationYPositive) {
    units::vec3 position = { 10.0f, 5.0f, 2.0f };
    units::vec3 velocity = { 0.0f, 0.0f, 0.0f };
    float deltaTime = 0.1f;
    controller->setTargetPosition(position[0], position[1] + 5, position[2]);
    auto result = controller->computeAccelerationRequest(position, velocity, deltaTime);

    EXPECT_TRUE(validateDirection(result.yAccel, direction::positive));
}

TEST_F(PIDDroneControllerFixture, ComputesExpectedAccelerationYNegative) {
    units::vec3 position = { 10.0f, 5.0f, 2.0f };
    units::vec3 velocity = { 0.0f, 0.0f, 0.0f };
    float deltaTime = 0.1f;
    controller->setTargetPosition(position[0], position[1] - 5, position[2]);
    auto result = controller->computeAccelerationRequest(position, velocity, deltaTime);

    EXPECT_TRUE(validateDirection(result.yAccel, direction::negative));
}

TEST_F(PIDDroneControllerFixture, ComputesExpectedAccelerationZPositive) {
    units::vec3 position = { 10.0f, 5.0f, 2.0f };
    units::vec3 velocity = { 0.0f, 0.0f, 0.0f };
    float deltaTime = 0.1f;
    controller->setTargetPosition(position[0], position[1], position[2] + 5);
    auto result = controller->computeAccelerationRequest(position, velocity, deltaTime);

    EXPECT_TRUE(validateDirection(result.zAccel, direction::positive));
}

TEST_F(PIDDroneControllerFixture, ComputesExpectedAccelerationZNegative) {
    units::vec3 position = { 10.0f, 5.0f, 2.0f };
    units::vec3 velocity = { 0.0f, 0.0f, 0.0f };
    float deltaTime = 0.1f;
    controller->setTargetPosition(position[0], position[1], position[2] - 5);
    auto result = controller->computeAccelerationRequest(position, velocity, deltaTime);

    EXPECT_TRUE(validateDirection(result.zAccel, direction::negative));
}

//verifies rotation direction

TEST_F(PIDDroneControllerFixture, ComputeBasisAOT_XAxisTest){
    quaternionVehicle vehicleState;
    auto orginalPose = vehicleState.getPose();

    float angleAroundX = 0.5f;

    vehicleState.eulerRotation(angleAroundX,0,0);
    auto newState = vehicleState.getPose();
    requestedVehicleState requestedState;
    requestedState.vehicleState = newState.dirVector;

    normalizeVectorInPlace(requestedState.vehicleState);
    
    auto result = controller->computeVehicleBasisAOT(orginalPose,requestedState);

    EXPECT_FLOAT_EQ(angleAroundX,-result.angleX);
}


TEST_F(PIDDroneControllerFixture, ComputeBasisAOT_XAxisTest_Negative){
    quaternionVehicle vehicleState;
    auto orginalPose = vehicleState.getPose();

    float angleAroundX = -0.5f;

    vehicleState.eulerRotation(angleAroundX,0,0);
    auto newState = vehicleState.getPose();
    requestedVehicleState requestedState;
    requestedState.vehicleState = newState.dirVector;

    normalizeVectorInPlace(requestedState.vehicleState);
    
    auto result = controller->computeVehicleBasisAOT(orginalPose,requestedState);

    EXPECT_FLOAT_EQ(angleAroundX,-result.angleX);
}

TEST_F(PIDDroneControllerFixture, ComputeBasisAOT_YAxisTest) {
    quaternionVehicle vehicleState;
    auto originalPose = vehicleState.getPose();

    float angleAroundY = 0.5f;

    vehicleState.eulerRotation(0, angleAroundY, 0);

    auto newState = vehicleState.getPose();
    requestedVehicleState requestedState;
    requestedState.vehicleState = newState.dirVector;

    normalizeVectorInPlace(requestedState.vehicleState);

    auto result = controller->computeVehicleBasisAOT(originalPose, requestedState);
    std::cout<< angleAroundY << ","<<-result.angleY <<"\n ";
    EXPECT_FLOAT_EQ(angleAroundY, -result.angleY);
}

TEST_F(PIDDroneControllerFixture, ComputeBasisAOT_YAxisTest_InvertedNegative) {
    quaternionVehicle vehicleState;

    float angleAroundY = -0.5f;

    vehicleState.eulerRotation(0, angleAroundY, 0);

    vehicleState.eulerRotation(M_PI,0,0);

    auto newState = vehicleState.getPose();
    requestedVehicleState requestedState;
    requestedState.vehicleState = newState.dirVector;

    quaternionVehicle originalVehicle;
    originalVehicle.eulerRotation(M_PI,0,0);
    auto originalPose = originalVehicle.getPose();
    normalizeVectorInPlace(requestedState.vehicleState);

    auto result = controller->computeVehicleBasisAOT(originalPose, requestedState);

    EXPECT_FLOAT_EQ(angleAroundY, -result.angleY);
}


TEST_F(PIDDroneControllerFixture, ComputeBasisAOT_XAxisTest_InvertedNegative) {
    quaternionVehicle vehicleState;

    float angleAroundX = -0.5f;

    vehicleState.eulerRotation(angleAroundX,0, 0);

    vehicleState.eulerRotation(M_PI,0,0);

    auto newState = vehicleState.getPose();
    requestedVehicleState requestedState;
    requestedState.vehicleState = newState.dirVector;

    quaternionVehicle originalVehicle;
    originalVehicle.eulerRotation(M_PI,0,0);
    auto originalPose = originalVehicle.getPose();
    normalizeVectorInPlace(requestedState.vehicleState);

    auto result = controller->computeVehicleBasisAOT(originalPose, requestedState);

    EXPECT_FLOAT_EQ(angleAroundX, -result.angleX);
}

TEST_F(PIDDroneControllerFixture, ComputesExpectedMoments) {
    units::vec3 position = { 10.0f, 5.0f, 2.0f };
    auto state = CoordinateSystem::WORLD_BASIS;
     controller->setTargetPosition(position[0]-5, position[1], position[2]);
    float deltaTime = 0.1f;

}
   

TEST_F(PIDDroneControllerFixture, angularVelocityXPositive){

    auto euler = controller->createEuler(0.5,0);
    float deltaTime = 0.01;
    auto rate = controller->computeAngularVelocityFromAOT(euler,deltaTime);


}

TEST_F(PIDDroneControllerFixture, angularVelocityXNegative){

    auto euler = controller->createEuler(-0.5,0);
    float deltaTime = 0.01;
    auto rate = controller->computeAngularVelocityFromAOT(euler,deltaTime);


}

TEST_F(PIDDroneControllerFixture, angularVelocityY){

    auto euler = controller->createEuler(0,0.5);
    float deltaTime = 0.01;
    auto rate = controller->computeAngularVelocityFromAOT(euler,deltaTime);

}


