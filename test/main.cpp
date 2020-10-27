#include "Logger.h"
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    auto default_logger = logger::setup_default_logger();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
