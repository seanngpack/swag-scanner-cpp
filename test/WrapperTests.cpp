#include <gtest/gtest.h>
#include "CoreBluetoothWrapper.h"


using namespace testing;


class FileHandlerFixture : public ::testing::Test {


protected:

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(FileHandlerFixture, TestWrapper) {
    void *p = get_wrapper_object();
    std::cout << "memory address of object: ";
    std::cout << p << std::endl;

//    int a = 5;
//    void *param = &a;void *p = get_shared_instance();


//
//    int x = MyObjectDoSomethingWith(p, param);
//    std::cout << x << std::endl;

//    initialize_bt2(p);

}