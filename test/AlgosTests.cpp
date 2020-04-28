#include "gtest/gtest.h"
#include <pcl/point_types.h>
#include "Algorithms.h"

class AlgosFixture : public ::testing::Test {

protected:
    camera::ss_intrinsics *intrinsics_no_distoration;
    camera::ss_intrinsics *intrinsics_distoration;
    std::vector<uint16_t> frame;

    virtual void SetUp() {
        float no_distortion[5] = {0,0,0,0,0};
        float distortion[5] = {.139, .124, .0043, .00067, -.034};
        intrinsics_no_distoration = new camera::ss_intrinsics(640, 480,
                                               475.07, 475.07,
                                               309.931, 245.011,
                                               "brown", no_distortion,
                                               0.0001);
        intrinsics_distoration = new camera::ss_intrinsics(640, 480,
                                                              475.07, 475.07,
                                                              309.931, 245.011,
                                                              "brown", distortion,
                                                              0.0001);


        for (float i = 0; i < intrinsics_no_distoration->width * intrinsics_no_distoration->height; i++) {
            frame.push_back(1);
        }
    }

    virtual void TearDown() {
        delete intrinsics_no_distoration;
        delete intrinsics_distoration;
    }


};

/**
 * Tests deprojection method to see if a point is being made correctly and see if the math
 * is good.
 */
TEST_F(AlgosFixture, TestDeprojectNoDistortion) {
    pcl::PointXYZ actual = algos::deproject_pixel_to_point(10, 10, 100, intrinsics_no_distoration);

    pcl::PointXYZ expected;
    expected.x = -.0063134059;
    expected.y = -.0049468712;
    expected.z = .01;

    ASSERT_FLOAT_EQ(expected.x, actual.x);
    ASSERT_FLOAT_EQ(expected.y, actual.y);
    ASSERT_FLOAT_EQ(expected.z, actual.z);
}

/**
 * Tests deprojection method with distortion coefficients.
 */
TEST_F(AlgosFixture, TestDeprojectDistortion) {
    pcl::PointXYZ actual = algos::deproject_pixel_to_point(10, 10, 100, intrinsics_distoration);

    pcl::PointXYZ expected;
    expected.x = -.0071082721;
    expected.y = -.0055454033;
    expected.z = .01;

    ASSERT_FLOAT_EQ(expected.x, actual.x);
    ASSERT_FLOAT_EQ(expected.y, actual.y);
    ASSERT_FLOAT_EQ(expected.z, actual.z);
}

/**
 * Test creating a pointcloud and see if its size, height, and width are good.
 */
TEST_F(AlgosFixture, TestCreatePC) {
    uint16_t *swag = frame.data();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = algos::create_point_cloud(swag, intrinsics_no_distoration);

    ASSERT_EQ(cloud->width, 640);
    ASSERT_EQ(cloud->height, 480);
    ASSERT_EQ(cloud->size(), 307200);
    ASSERT_TRUE(cloud->is_dense);
}
