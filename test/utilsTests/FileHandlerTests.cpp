#include <gtest/gtest.h>
#include "FileHandler.h"
#include "json.hpp"

using json = nlohmann::json;
using namespace testing;


class FileHandlerFixture : public ::testing::Test {


protected:
    file::FileHandler *handler;

    virtual void SetUp() {
        handler = new file::FileHandler(true);
    }

    virtual void TearDown() {
        delete handler;
    }
};

// remember to watch out for creating a brand new directory from scratch. See if
// all folders are being created, files, etc.

/**
 * Test to see if the method sets the correct current scan folder.
 * Again, if there are folders 1->10 in the valid directory, then the current
 * scan folder should be 11.
 * NOTE: This test is flawed because when I change the default folder this test
 * will break. Figure out a solution later to address this.
 */
TEST_F(FileHandlerFixture, TestGetCurrentScanFolder) {
    EXPECT_EQ(handler->get_scan_folder_path(),
              "/Users/seanngpack/Library/Application Support/SwagScanner/scans/1");
}

/**
 * Test to see if passing "true" to the constructor will make a new folder
 * Skip this test when not in use.
 */
TEST_F(FileHandlerFixture, TestConstructor2) {
    GTEST_SKIP();
    file::FileHandler *handler = new file::FileHandler(true);
}

/**
 * Test to see if passing a scan name will work.
 */
TEST_F(FileHandlerFixture, TestConstructor3) {
    GTEST_SKIP();
    file::FileHandler *handler = new file::FileHandler("swagg");
}

/**
 * Note: disable this test when not in use.
 * Test setting the current folder to something valid.
 * Also see if it creates new folders within it.
 */
TEST_F(FileHandlerFixture, TestSetFolderPath) {
//    handler->set_scan_folder_path("/Users/seanngpack/Programming Stuff");
//    EXPECT_EQ(handler->get_scan_folder_path(),
//              "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing");
}

/**
 * Test setting the current folder to something invalid.
 */
TEST_F(FileHandlerFixture, TestSetFolderPathInvalid) {
    EXPECT_THROW(handler->set_scan_folder_path("aboslute nononsense"),
                 std::invalid_argument);
}
