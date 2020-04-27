#include "gtest/gtest.h"
#include <pcl/point_types.h>
#include "Algorithms.h"

class AlgosFixture : public ::testing::Test {

protected:
    camera::ss_intrinsics *intrinsics;

    virtual void SetUp() {
        intrinsics = new camera::ss_intrinsics(50, 50,
                                               50, 50,
                                               50, 50,
                                               "brown", 10, .01);
    }

    virtual void TearDown() {
        delete intrinsics;
    }


};

/**
 * Tests deprojection method to see if a point is being made correctly and see if the math
 * is good.
 */
TEST_F(AlgosFixture, TestDeproject) {
    pcl::PointXYZ actual = algos::deproject_pixel_to_point(10, 10, 100, intrinsics);

    pcl::PointXYZ expected;
    expected.x = 9;
    expected.y = 9;
    expected.z = 1;

    ASSERT_FLOAT_EQ(expected.x, actual.x);
    ASSERT_FLOAT_EQ(expected.y, actual.y);
    ASSERT_FLOAT_EQ(expected.z, actual.z);
}
