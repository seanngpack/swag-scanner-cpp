#include <gtest/gtest.h>
#include "FileHandler.h"

using namespace testing;


class FileHandlerFixture : public ::testing::Test {


protected:
    file::FileHandler *handler;

    virtual void SetUp() {
        handler = new file::FileHandler;
    }

    virtual void TearDown() {
        delete handler;
    }
};


/**
 * Comment this test out when not in use.
 * Tests to see if the constructor actually creates a new folder when
 * the auto_create flag is set to true.
 */
//TEST_F(FileHandlerFixture, TestCreateFolderWithConstructor) {
//    file::FileHandler *test = new file::FileHandler(
//            "/Users/seanngpack/Programming Stuff/Projects/scanner_files",
//            true);
//}

/**
 * Test the get current scan folder method with an invalid path name.
 * Expecting invalid argument errors.
 */
TEST_F(FileHandlerFixture, TestGetCurrentScanFolderInvalidPath) {
    ASSERT_THROW(file::FileHandler *test = new file::FileHandler("swagggg"),
                 std::invalid_argument);
}

/**
 * Test to see if the method sets the correct current scan folder.
 * Again, if there are folders 1->10 in the valid directory, then the current
 * scan folder should be 11.
 * NOTE: This test is flawed because when I change the default folder this test
 * will break. Figure out a solution later to address this.
 */
TEST_F(FileHandlerFixture, TestGetCurrentScanFolder) {
    EXPECT_EQ(handler->get_scan_folder_path(),
              "/Users/seanngpack/Programming Stuff/Projects/scanner_files/17");
}

/**
 * Test if the file handler is smart enough to make the current scan directory "1" if
 * the directory entereed is empty.
 */
TEST_F(FileHandlerFixture, TestGetCurrentScanFolderEmptyCase) {
    file::FileHandler *test = new file::FileHandler(
            "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing/empty");
    EXPECT_EQ(test->get_scan_folder_path(),
              "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing/empty/1");
}

/**
 * Note: disable this test when not in use.
 * Test setting the current folder to something valid.
 * Also see if it creates new folders within it.
 */
TEST_F(FileHandlerFixture, TestSetFolderPath) {
//    handler->set_scan_folder_path("/Users/seanngpack/Programming Stuff");
//    EXPECT_EQ(handler->get_scan_folder_path(),
//              "/Users/seanngpack/Programming Stuff");
}

/**
 * Test setting the current folder to something invalid.
 */
TEST_F(FileHandlerFixture, TestSetFolderPathInvalid) {
    EXPECT_THROW(handler->set_scan_folder_path("aboslute nononsense"),
                 std::invalid_argument);
}