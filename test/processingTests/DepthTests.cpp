#include "gtest/gtest.h"
#include <pcl/point_types.h>
#include "Depth.h"
#include <librealsense2/h/rs_types.h>
#include "CameraTypes.h"


class DepthFixture : public ::testing::Test {

protected:
    camera::intrinsics *intrinsics_no_distoration;
    camera::intrinsics *intrinsics_distoration;
    std::vector<uint16_t> frame;

    virtual void SetUp() {
        float no_distortion[5] = {0, 0, 0, 0, 0};
        float distortion[5] = {.139, .124, .0043, .00067, -.034};
        intrinsics_no_distoration = new camera::intrinsics(640, 480,
                                                           475.07, 475.07,
                                                           309.931, 245.011,
                                                           RS2_DISTORTION_INVERSE_BROWN_CONRADY,
                                                           no_distortion,
                                                           0.0001);
        intrinsics_distoration = new camera::intrinsics(640, 480,
                                                        475.07, 475.07,
                                                        309.931, 245.011,
                                                        RS2_DISTORTION_INVERSE_BROWN_CONRADY, distortion,
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
 * Test creating a pointcloud and see if its size, height, and width are good.
 */
TEST_F(DepthFixture, TestCreatePC) {
    const uint16_t *swag = frame.data();
    std::vector<uint16_t> depth_frame(swag, swag + sizeof swag / sizeof swag[0]);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud = depth::create_point_cloud(depth_frame, *intrinsics_no_distoration);

    ASSERT_EQ(cloud->width, 640);
    ASSERT_EQ(cloud->height, 480);
    ASSERT_EQ(cloud->size(), 307200);
    ASSERT_TRUE(cloud->is_dense);
}
