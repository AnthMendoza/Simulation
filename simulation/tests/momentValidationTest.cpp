#include <gtest/gtest.h>
#include "../include/dynamics/vehicle.h"
#include <string>
#include "../include/core/coordinateSystem.h"



using namespace SimCore;

class momentValidationFixture : public ::testing::Test {
protected:
    std::string droneConfig = "../tests/testConfigs/drone_test_config.toml";    

    class testVehicle : public Vehicle{
    private:

    public:


    void init(string& config){
        Vehicle::init(config);
    }

    void setEntitiesPose(const poseState& pose){

    }

    };

    std::unique_ptr<testVehicle> vehicle;

    void SetUp() override {
        vehicle = std::make_unique<testVehicle>();
        droneConfig = readFileAsString(droneConfig);
        vehicle->init(droneConfig);
        auto basisPose = CoordinateSystem::WORLD_BASIS;
        vehicle->setStateVector(basisPose.dirVector,basisPose.fwdVector);
    }

    void TearDown() override {
        vehicle.reset();
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

static constexpr float EPSILON = 0.0001;

TEST_F(momentValidationFixture,ValidateRotationDirectionXPositive){
    
    threeDState moments = {1,0,0};
    float timeStep = 0.01f;
    for(int i = 0 ; i < 10 ; i++){
        vehicle->addMoment(moments);
        vehicle->updateState(timeStep * i);
    }

    auto rotatedPose = vehicle->getPose();

    auto basisPose = CoordinateSystem::WORLD_BASIS;
    auto basisFwdVector = basisPose.fwdVector;
    auto fwdVector = rotatedPose.fwdVector;

    EXPECT_NEAR(fwdVector[0],basisFwdVector[0],EPSILON);
    EXPECT_NEAR(fwdVector[1],basisFwdVector[1],EPSILON);
    EXPECT_NEAR(fwdVector[2],basisFwdVector[2],EPSILON);

    auto dirVector = rotatedPose.dirVector;

    EXPECT_NEAR(dirVector[0],0.0f,EPSILON);
    EXPECT_TRUE(validateDirection(dirVector[1],direction::negative));
    

}


TEST_F(momentValidationFixture,ValidateRotationDirectionXNegative){
    
    threeDState moments = {-1,0,0};
    float timeStep = 0.01f;
    for(int i = 0 ; i < 10 ; i++){
        vehicle->addMoment(moments);
        vehicle->updateState(timeStep * i);
    }

    auto rotatedPose = vehicle->getPose();

    auto basisPose = CoordinateSystem::WORLD_BASIS;
    auto basisFwdVector = basisPose.fwdVector;
    auto fwdVector = rotatedPose.fwdVector;

    EXPECT_NEAR(fwdVector[0],basisFwdVector[0],EPSILON);
    EXPECT_NEAR(fwdVector[1],basisFwdVector[1],EPSILON);
    EXPECT_NEAR(fwdVector[2],basisFwdVector[2],EPSILON);

    auto dirVector = rotatedPose.dirVector;

    EXPECT_NEAR(dirVector[0],0.0f,EPSILON);
    EXPECT_TRUE(validateDirection(dirVector[1],direction::positive));
    

}


TEST_F(momentValidationFixture,ValidateRotationDirectionYPositive){
    
    threeDState moments = {0,1,0};
    float timeStep = 0.01f;
    for(int i = 0 ; i < 10 ; i++){
        vehicle->addMoment(moments);
        vehicle->updateState(timeStep * i);
    }

    auto rotatedPose = vehicle->getPose();

    auto basisPose = CoordinateSystem::WORLD_BASIS;
    auto basisRightVector = basisPose.rightVector;
    auto rightVector = rotatedPose.rightVector;

    EXPECT_NEAR(rightVector[0],basisRightVector[0],EPSILON);
    EXPECT_NEAR(rightVector[1],basisRightVector[1],EPSILON);
    EXPECT_NEAR(rightVector[2],basisRightVector[2],EPSILON);

    auto dirVector = rotatedPose.dirVector;

    EXPECT_NEAR(dirVector[1],0.0f,EPSILON);
    EXPECT_TRUE(validateDirection(dirVector[0],direction::positive));
    

}

TEST_F(momentValidationFixture,ValidateRotationDirectionYNegative){
    
    threeDState moments = {0,-1,0};
    float timeStep = 0.01f;
    for(int i = 0 ; i < 10 ; i++){
        vehicle->addMoment(moments);
        vehicle->updateState(timeStep * i);
    }

    auto rotatedPose = vehicle->getPose();

    auto basisPose = CoordinateSystem::WORLD_BASIS;
    auto basisRightVector = basisPose.rightVector;
    auto rightVector = rotatedPose.rightVector;

    EXPECT_NEAR(rightVector[0],basisRightVector[0],EPSILON);
    EXPECT_NEAR(rightVector[1],basisRightVector[1],EPSILON);
    EXPECT_NEAR(rightVector[2],basisRightVector[2],EPSILON);

    auto dirVector = rotatedPose.dirVector;

    EXPECT_NEAR(dirVector[1],0.0f,EPSILON);
    EXPECT_TRUE(validateDirection(dirVector[0],direction::negative));
    

}


TEST_F(momentValidationFixture,ValidateRotationDirectionZPositive){
    
    threeDState moments = {0,0,1};
    float timeStep = 0.01f;
    for(int i = 0 ; i < 10 ; i++){
        vehicle->addMoment(moments);
        vehicle->updateState(timeStep * i);
    }

    auto rotatedPose = vehicle->getPose();

    auto basisPose = CoordinateSystem::WORLD_BASIS;
    auto basisDirVector = basisPose.dirVector;
    auto dirVector = rotatedPose.dirVector;

    EXPECT_NEAR(dirVector[0],basisDirVector[0],EPSILON);
    EXPECT_NEAR(dirVector[1],basisDirVector[1],EPSILON);
    EXPECT_NEAR(dirVector[2],basisDirVector[2],EPSILON);

    auto fwdVector = rotatedPose.fwdVector;

    EXPECT_NEAR(fwdVector[2],0.0f,EPSILON);
    EXPECT_TRUE(validateDirection(fwdVector[1],direction::positive));
    

}


TEST_F(momentValidationFixture,ValidateRotationDirectionZNegative){
    
    threeDState moments = {0,0,-1};
    float timeStep = 0.01f;
    for(int i = 0 ; i < 10 ; i++){
        vehicle->addMoment(moments);
        vehicle->updateState(timeStep * i);
    }

    auto rotatedPose = vehicle->getPose();

    auto basisPose = CoordinateSystem::WORLD_BASIS;
    auto basisDirVector = basisPose.dirVector;
    auto dirVector = rotatedPose.dirVector;

    EXPECT_NEAR(dirVector[0],basisDirVector[0],EPSILON);
    EXPECT_NEAR(dirVector[1],basisDirVector[1],EPSILON);
    EXPECT_NEAR(dirVector[2],basisDirVector[2],EPSILON);

    auto fwdVector = rotatedPose.fwdVector;

    EXPECT_NEAR(fwdVector[2],0.0f,EPSILON);
    EXPECT_TRUE(validateDirection(fwdVector[1],direction::negative));
    

}
