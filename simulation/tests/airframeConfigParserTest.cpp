#include <gtest/gtest.h>

#include <filesystem>

#include <airframe_config.h>

TEST(AirframeConfigParser, ParsesDefaultsAndActuatorOverrides) {
    const std::string config = R"(
[component_library]
motor = "motor.toml"
propeller = "propeller.toml"

[[actuator]]
name = "front_right"
position = [0.15, 0.15, 0.0]
thrustDirection = [0.0, 0.0, 1.0]
rotationDirection = -1

[[actuator]]
name = "front_left"
position = [0.15, -0.15, 0.0]
thrustDirection = [0.0, 0.1, 0.99]
rotationDirection = 1
motor = "/tmp/custom_motor.toml"
propeller = "custom_propeller.toml"
)";

    const std::filesystem::path baseDirectory("/tmp/airframe");
    const auto parsed = avionics::airframe_config_parser::fromToml(config, baseDirectory);

    ASSERT_EQ(parsed.actuators.size(), 2);
    EXPECT_EQ(parsed.defaultMotorConfigPath, "/tmp/airframe/motor.toml");
    EXPECT_EQ(parsed.defaultPropellerConfigPath, "/tmp/airframe/propeller.toml");

    EXPECT_EQ(parsed.actuators[0].name, "front_right");
    EXPECT_EQ(parsed.actuators[0].motorConfigPath, "/tmp/airframe/motor.toml");
    EXPECT_EQ(parsed.actuators[0].propellerConfigPath, "/tmp/airframe/propeller.toml");
    EXPECT_FLOAT_EQ(parsed.actuators[0].position[0], 0.15f);
    EXPECT_FLOAT_EQ(parsed.actuators[0].position[1], 0.15f);
    EXPECT_FLOAT_EQ(parsed.actuators[0].position[2], 0.0f);
    EXPECT_EQ(parsed.actuators[0].rotationDirection, -1);

    EXPECT_EQ(parsed.actuators[1].name, "front_left");
    EXPECT_EQ(parsed.actuators[1].motorConfigPath, "/tmp/custom_motor.toml");
    EXPECT_EQ(parsed.actuators[1].propellerConfigPath, "/tmp/airframe/custom_propeller.toml");
    EXPECT_FLOAT_EQ(parsed.actuators[1].thrustDirection[0], 0.0f);
    EXPECT_FLOAT_EQ(parsed.actuators[1].thrustDirection[1], 0.1f);
    EXPECT_FLOAT_EQ(parsed.actuators[1].thrustDirection[2], 0.99f);

    const auto positions = parsed.motorPositions();
    const auto thrustDirections = parsed.thrustDirections();
    const auto rotationDirections = parsed.rotationDirections();

    ASSERT_EQ(positions.size(), 2);
    ASSERT_EQ(thrustDirections.size(), 2);
    ASSERT_EQ(rotationDirections.size(), 2);
    EXPECT_FLOAT_EQ(rotationDirections[0], -1.0f);
    EXPECT_FLOAT_EQ(rotationDirections[1], 1.0f);
}

TEST(AirframeConfigParser, RequiresActuators) {
    const std::string config = R"(
[component_library]
motor = "motor.toml"
propeller = "propeller.toml"
)";

    EXPECT_THROW(
        avionics::airframe_config_parser::fromToml(config, "/tmp/airframe"),
        std::runtime_error
    );
}
