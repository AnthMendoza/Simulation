#include <util/yaml.h>
#include <gtest/gtest.h>

using namespace utility;

TEST(GetRequiredTests, RetrievesValidInt) {
    YAML::Node node = YAML::Load(R"(
        value: 42
    )");

    int result = getRequired<int>(node, "value");

    EXPECT_EQ(result, 42);
}

TEST(GetRequiredTests, RetrievesValidString) {
    YAML::Node node = YAML::Load(R"(
        name: "test"
    )");

    std::string result = getRequired<std::string>(node, "name");

    EXPECT_EQ(result, "test");
}

TEST(GetRequiredTests, ThrowsOnMissingKey) {
    YAML::Node node = YAML::Load(R"(
        value: 10
    )");

    EXPECT_THROW(
        getRequired<int>(node, "missing"),
        std::runtime_error
    );
}

TEST(GetRequiredTests, ThrowsOnInvalidType) {
    YAML::Node node = YAML::Load(R"(
        value: "not_an_int"
    )");

    EXPECT_THROW(
        getRequired<int>(node, "value"),
        std::runtime_error
    );
}
