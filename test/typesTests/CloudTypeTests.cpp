#include "gtest/gtest.h"
#include "CloudType.h"


TEST(CloudTypeTests, TestEnumMap) {
    ASSERT_EQ(CloudType::String(CloudType::Type::RAW), "raw");
}

TEST(CloudTypeTests, TestEnumForLoop) {
    std::vector<std::string> v;
    for ( const auto e : CloudType::All )
        v.push_back(CloudType::String(e));

    ASSERT_EQ(v[0], "raw");
    ASSERT_EQ(v[1], "filtered");
    ASSERT_EQ(v[2], "segmented");
    ASSERT_EQ(v[3], "normal");

}