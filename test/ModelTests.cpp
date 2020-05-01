#include "gtest/gtest.h"
#include "Model.h"

class ModelFixture : public ::testing::Test {

protected:
    std::vector<uint16_t> depth_frame;
    const uint16_t *depth_frame_ptr;
    model::Model *mod;

    virtual void SetUp() {
        mod = new model::Model;
        for (uint16_t i = 0; i < 100; i++) {
            depth_frame.push_back(i);
        }
        depth_frame_ptr = depth_frame.data();
    }

    virtual void TearDown() {
        delete mod;
    }
};


TEST_F(ModelFixture, TestSetDepthFrame) {
    int x = 0;
    mod->set_depth_frame(depth_frame_ptr);
    ASSERT_EQ(0, depth_frame_ptr[0]);
}
