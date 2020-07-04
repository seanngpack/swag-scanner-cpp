#ifndef SWAG_SCANNER_NORMALTESTS_CPP
#define SWAG_SCANNER_NORMALTESTS_CPP

#include <gtest/gtest.h>
#include <iostream>
#include "Normal.h"

TEST(NormalTests, TestConstructor1) {
    equations::Normal n(1.1, 1.3, -1.1);
    EXPECT_FLOAT_EQ(n.A, 1.1);
    EXPECT_FLOAT_EQ(n.B, 1.3);
    EXPECT_FLOAT_EQ(n.C, -1.1);
}

TEST(NormalTests, TestConstructor2) {
    std::vector<double> v{1.1, 1.3, -1.1};
    equations::Normal n(v);
    EXPECT_FLOAT_EQ(n.A, 1.1);
    EXPECT_FLOAT_EQ(n.B, 1.3);
    EXPECT_FLOAT_EQ(n.C, -1.1);
}

#endif //SWAG_SCANNER_NORMALTESTS_CPP