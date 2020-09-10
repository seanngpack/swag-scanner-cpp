#include <gtest/gtest.h>
#include <memory>
#include "Model.h"
#include "Visualizer.h"
#include "Calibration.h"
#include "Point.h"



class CalibrationPhysicalFixture : public ::testing::Test {

protected:
    model::Model *mod;
    equations::Normal g_n = equations::Normal(-0.0158, -0.8661, -0.4996);
    std::vector<equations::Plane> planes = {
            equations::Plane(0.8603, -0.2446, 0.4472, -.201376),
            equations::Plane(0.7779, -0.3059, 0.5489, -.242753),
            equations::Plane(0.6714, -0.3638, 0.6457, -.28239),
            equations::Plane(-0.5484, -0.4098, 0.7289, -.316794),
            equations::Plane(-0.4123, -0.4443, 0.7954, -.344772),
            equations::Plane(-0.2635, -0.4709, 0.8419, -.364747),
            equations::Plane(-0.1081, -0.4863, 0.8670, -.376452),
            equations::Plane(-0.0492, -0.4900, 0.8703, -.379304),
            equations::Plane(-0.2061, -0.4751, 0.8555, -.374743),
            equations::Plane(-0.3655, -0.4537, 0.8128, -.359423)
    };

    virtual void SetUp() {
        mod = new model::Model();
    }

    virtual void TearDown() {
        delete mod;
    }
};

/**
 * Make sure the center point calculation is good.
 */
TEST_F(CalibrationPhysicalFixture, calculate_center_pt) {
    Eigen::MatrixXd A = calibration::build_A_matrix(g_n, planes);
    Eigen::MatrixXd b = calibration::build_b_matrix(g_n, planes);
    equations::Point pt = calibration::calculate_center_pt(A, b);
    ASSERT_NEAR(pt.x, -0.000213082, .001);
    ASSERT_NEAR(pt.y, 0.0298714, .001);
    ASSERT_NEAR(pt.z, 0.42673, .001);
}


/**
 * Test building the A matrix
 */
TEST_F(CalibrationPhysicalFixture, TestBuildAMatrix) {

    Eigen::MatrixXd A = calibration::build_A_matrix(g_n, planes);
    ASSERT_NEAR(A(0, 0), 0.08251, .001);
    ASSERT_NEAR(A(3, 2), -0.0664, .001);
    ASSERT_NEAR(A(8, 2), 0.0427, .001);
}

/**
 * Test building the b matrix
 */
TEST_F(CalibrationPhysicalFixture, TestBuildbMatrix) {

    Eigen::MatrixXd b = calibration::build_b_matrix(g_n, planes);
    ASSERT_NEAR(b(0, 0), -0.0414, .001);
    ASSERT_NEAR(b(5, 0), -0.0117, .001);
    ASSERT_NEAR(b(8, 0), 0.0153, .001);
}
