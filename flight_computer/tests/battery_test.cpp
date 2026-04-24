
#include <gtest/gtest.h>
#include <battery.h>
#include <memory>
#include <string>

using namespace hardware;

class BatteryFixture : public ::testing::Test {
protected:
    std::string config;
    std::shared_ptr<hardware::battery> bat;

    void SetUp() override {
        config = std::string(TEST_DIR);
        config = config + "/tests/battery.yaml";
        bat = std::make_shared<hardware::battery>(config);
    }


};

TEST_F(BatteryFixture, initialSocIsValid) {
    EXPECT_GE(bat->getSOC(), 0);
    EXPECT_LE(bat->getSOC(), 100);
}


TEST_F(BatteryFixture, socNeverGoesNegative) {
    for (int t = 1; t <= 10000; t++)
        bat->updateBattery(100.0f, (float)t);
    EXPECT_GE(bat->getSOC(), 0);
}


TEST_F(BatteryFixture, firstUpdateInitializesSOC) {
    float initialSoc = bat->getSOC();
    float cellVoltage = 4.0;
    bat->manuallySetInitVoltage(bat->getCellCount()*cellVoltage);

    EXPECT_NE(bat->getSOC(), initialSoc);
    EXPECT_GE(bat->getSOC(), 0);
    EXPECT_LE(bat->getSOC(), 100);
}


TEST_F(BatteryFixture, capacityDecreasesWithDischarge) {
    float initialCapacity = bat->getRemainingCapacityAh();

    bat->updateBattery(10.0f, 1.0f);
    bat->updateBattery(10.0f, 2.0f);

    EXPECT_LT(bat->getRemainingCapacityAh(), initialCapacity);
}


TEST_F(BatteryFixture, deadFlagTriggersAtZeroCapacity) {
    for (int t = 1; t <= 100000; t++){
        bat->updateBattery(100.0f, (float)t);
    }

    EXPECT_FALSE(bat->isCharged());
    EXPECT_NEAR(bat->getRemainingCapacityAh(), 0.01f, 1e-5);
}

TEST_F(BatteryFixture, negativeDeltaTime) {
    bat->updateBattery(10.0f, 10.0f);
    float cap_before = bat->getRemainingCapacityAh();
    bat->updateBattery(10.0f, 5.0f);

    EXPECT_LE(bat->getRemainingCapacityAh(), cap_before);
}

TEST_F(BatteryFixture, socNeverExceeds100) {
    for (int t = 1; t <= 100; t++)
        bat->updateBattery(-100.0f, (float)t);

    EXPECT_LE(bat->getSOC(), 100);
}


TEST_F(BatteryFixture, safetyTerminationDisablesCharge) {
    for (int t = 1; t <= 100000; t++)
        bat->updateBattery(100.0f, (float)t);

    EXPECT_LE(bat->getSOC(), bat->getSafetyTerminationLevel());
    EXPECT_FALSE(bat->isCharged());
}

TEST_F(BatteryFixture, voltageClampsToZeroUnderHighLoad) {
    bat->updateBattery(0.0f, 1.0f);

    bat->updateBattery(1e6f, 2.0f);

    EXPECT_EQ(bat->getVoltage(), 0);
}

TEST_F(BatteryFixture, voltageDropsWithHigherCurrent) {
    float cellVoltage = 4.0;

    bat->manuallySetInitVoltage(bat->getCellCount() * cellVoltage);
    
    bat->updateBattery(0.0f, 1.0f);
    float v_no_load = bat->getVoltage();

    bat->updateBattery(50.0f, 2.0f);
    float v_load = bat->getVoltage();

    EXPECT_LT(v_load, v_no_load);
}