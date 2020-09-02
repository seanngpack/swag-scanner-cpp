#include "gtest/gtest.h"
#include "../../src/types/CloudType.h"


TEST(CloudTypeTests, TestEnumMap) {
    ASSERT_EQ(CloudType::String(CloudType::Type::RAW), "raw");
}

TEST(CloudTypeTests, TestEnumForLoop) {
    std::vector<std::string> v;
    for ( const auto e : CloudType::All )
        v.push_back(CloudType::String(e));

    ASSERT_EQ(v[0], "raw");
    ASSERT_EQ(v[1], "filtered");
    ASSERT_EQ(v[2], "processed");
    ASSERT_EQ(v[3], "normal");
    ASSERT_EQ(v[4], "calibration");

}