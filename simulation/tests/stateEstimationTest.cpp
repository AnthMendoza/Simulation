
#include <gtest/gtest.h>
#include <cmath>
#include "../include/control/stateEstimation.h"
#include "../include/core/quaternion.h"
#include "../include/core/vectorMath.h"

using namespace SimCore;


class stateEstimationTestFixture : public ::testing::Test {
protected:

    class TestableStateEstimation : public stateEstimation {
    public:
        TestableStateEstimation(float timeBetweenUpdates) 
            : stateEstimation(timeBetweenUpdates) {}
        
 

        void setControl(const controllerToEstimationVariant& controlPacket) {
            control = controlPacket;
        }

        
    };

    void SetUp() override {

        estimator = std::make_unique<TestableStateEstimation>(0.01f);
    }

    void TearDown() override {
        estimator.reset();
    }

    std::unique_ptr<TestableStateEstimation> estimator;
};





TEST_F(stateEstimationTestFixture, GravityVectorEstimation){
    threeDState accel{0.0f, 0.0f, -9.81f};
    float time = 0.1f;
    onlyThrustAccel thrust;
    estimator->setControl(thrust);
    threeDState gravityVec = estimator->gravityVectorEstimation(accel, time);

    bool ans = similarVector(accel,gravityVec,0.1f);
    EXPECT_TRUE(ans);
}

TEST_F(stateEstimationTestFixture, GravityVectorEstimationSideAccel){
    threeDState accel{0.0f, 4.0f, -9.81f}; 
    float time = 0.1f;
    onlyThrustAccel thrust;
    thrust.thrustAccelerationVector[1] = 4.0;
    estimator->setControl(thrust);
    threeDState gravityVec = estimator->gravityVectorEstimation(accel, time);

    bool ans = similarVector({0,0,-9.8},gravityVec,0.1f);
    EXPECT_TRUE(ans);
}