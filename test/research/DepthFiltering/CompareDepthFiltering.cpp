#include <gtest/gtest.h>
#include "Model.h"
#include "Visualizer.h"

class CompareDepthFilteringFixture : public ::testing::Test {

protected:
    model::Model *mod;
    visual::Visualizer *viewer;

    virtual void SetUp() {
        mod = new model::Model();
        viewer = new visual::Visualizer();
    }

    virtual void TearDown() {
        delete mod;
        delete viewer;
    }
};

TEST_F(CompareDepthFilteringFixture, CompareCalFixture) {

}