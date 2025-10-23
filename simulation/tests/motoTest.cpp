#include <gtest/gtest.h>
#include "../../include/subsystems/motor.h"
#include "../../include/utility/utility.h"

using namespace SimCore;

class motorTestFixture : public ::testing::Test {
protected:
    std::unique_ptr<motor> testMotor;
    
    void SetUp() override {
        std::string testConfig = "../motor_test_config.toml";
        testConfig = readFileAsString(testConfig);
        testMotor = std::make_unique<motor>(testConfig);
    }

    void TearDown() override {
        testMotor.reset();
    }
};



TEST_F(motorTestFixture, GeneratesTorqueProportionalToCurrent) {

}
