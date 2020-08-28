#ifndef SWAG_SCANNER_PLANETESTS_CPP
#define SWAG_SCANNER_PLANETESTS_CPP

#include <gtest/gtest.h>
#include <iostream>
#include "../../src/model/equations/Plane.h"

TEST(PlaneTests, TestConstructor1) {
    equations::Plane p(1.1, 1.3, -1.1, 3.1);
    EXPECT_FLOAT_EQ(p.A, 1.1);
    EXPECT_FLOAT_EQ(p.B, 1.3);
    EXPECT_FLOAT_EQ(p.C, -1.1);
    EXPECT_FLOAT_EQ(p.D, 3.1);
}

TEST(PlaneTests, TestConstructor2) {
    std::vector<double> v{1.1, 1.3, -1.1, 3.1};
    equations::Plane p(v);
    EXPECT_FLOAT_EQ(p.A, 1.1);
    EXPECT_FLOAT_EQ(p.B, 1.3);
    EXPECT_FLOAT_EQ(p.C, -1.1);
    EXPECT_FLOAT_EQ(p.D, 3.1);
}

TEST(PlaneTests, TestGetNormal) {
    std::vector<double> v{1.1, 1.3, -1.1, 3.1};
    equations::Plane p(v);
    equations::Normal n = p.get_normal();
    EXPECT_FLOAT_EQ(n.A, 1.1);
    EXPECT_FLOAT_EQ(n.B, 1.3);
    EXPECT_FLOAT_EQ(n.C, -1.1);
}

TEST(PlaneTests, TestAdditionOverload) {
    std::vector<double> v{1.1, 1.3, -1.1, 3.1};
    equations::Plane p1(v);
    equations::Plane p2(v);
    equations::Plane p3 = p1 + p2;
    EXPECT_DOUBLE_EQ(p3.A, 2.2);
    EXPECT_DOUBLE_EQ(p3.B, 2.6);
    EXPECT_DOUBLE_EQ(p3.C, -2.2);
    EXPECT_DOUBLE_EQ(p3.D, 6.2);
}

#endif //SWAG_SCANNER_PLANETESTS_CPP