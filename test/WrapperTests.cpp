#include <gtest/gtest.h>
#include "CoreBluetoothWrapper.h"


using namespace testing;


class FileHandlerFixture : public ::testing::Test {


//protected:
//    file::FileHandler *handler;
//
//    virtual void SetUp() {
//        handler = new file::FileHandler;
//    }
//
//    virtual void TearDown() {
//        delete handler;
//    }
};

TEST_F(FileHandlerFixture, TestWrapper) {
    void * p = get_wrapper_object();
    std::cout << p << std::endl;

    int a = 5;
    void * param = &a;

    int x = MyObjectDoSomethingWith(p, param);
    std::cout << x << std::endl;
}