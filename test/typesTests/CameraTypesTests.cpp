#include "../../src/types/CameraTypes.h"
#include "gtest/gtest.h"


class CameraTypesFixture : public ::testing::Test {


protected:
    camera::ss_intrinsics *intrinsics;

    virtual void SetUp() {
        float distortion[5] = {.139, .124, .0043, .00067, -.034};
        intrinsics = new camera::ss_intrinsics(100, 100,
                                               10.0, 10.0, 100.0, 100.0,
                                               RS2_DISTORTION_INVERSE_BROWN_CONRADY,
                                               distortion, .001);
    }

    virtual void TearDown() {
        delete intrinsics;
    }
};


TEST_F(CameraTypesFixture, TestToString) {
    ASSERT_EQ(intrinsics->to_string(), "width: 100\n"
                                      "height: 100\n"
                                      "fx: 10.000000\n"
                                      "fy: 10.000000\n"
                                      "ppx: 100.000000\n"
                                      "ppy: 100.000000\n"
                                      "depth scale: 0.001000");
}
