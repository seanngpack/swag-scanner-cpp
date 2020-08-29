//#include <gtest/gtest.h>
//#include "ScanFileHandler.h"
//#include <nlohmann/json.hpp>
//
//using json = nlohmann::json;
//using namespace testing;
//
//
//class FileHandlerFixture : public ::testing::Test {
//
//
//protected:
//    file::ScanFileHandler *handler;
//
//    virtual void SetUp() {
//        handler = new file::ScanFileHandler();
//    }
//
//    virtual void TearDown() {
//        delete handler;
//    }
//};
//
//// remember to watch out for creating a brand new directory from scratch. See if
//// all folders are being created, files, etc.
//
//
///**
// * Test to see if passing "true" to the constructor will make a new folder
// * Skip this test when not in use.
// */
//TEST_F(FileHandlerFixture, TestConstructor2) {
//    GTEST_SKIP();
//    file::IFileHandler *handler = new file::ScanFileHandler(true);
//}
//
///**
// * Test to see if passing a scan name will work.
// */
//TEST_F(FileHandlerFixture, TestConstructor3) {
//    GTEST_SKIP();
//    file::IFileHandler *handler = new file::ScanFileHandler("swagg");
//}
//
///**
// * Note: disable this test when not in use.
// * Test setting the current folder to something valid.
// * Also see if it creates new folders within it.
// */
//TEST_F(FileHandlerFixture, TestSetFolderPath) {
////    handler->set_scan_folder_path("/Users/seanngpack/Programming Stuff");
////    EXPECT_EQ(handler->get_scan_folder_path(),
////              "/Users/seanngpack/Programming Stuff/Projects/scanner_files/testing");
//}
//
