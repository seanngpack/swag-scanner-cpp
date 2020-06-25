///**
// * Test the wrapper on a main method to test async actions. Can't really unit test
// * this bad boy.
// */
//#include <iostream>
//#include <thread>
//#include <Arduino.h>
//
//using namespace std::literals::chrono_literals;
//
//
//int main() {
//    auto *a = new arduino::Arduino;
//    std::cout << "main thread: " << std::this_thread::get_id() << std::endl;
//    a->rotate_table(10);
//    a->rotate_table(10);
//    a->rotate_table(10);
//
//    delete a;
//
//
////    // keep the main thread alive
////    while (true) {
////        std::this_thread::sleep_for(1s);
//////        std::cout << "working..." << std::endl;
////    }
//
//    return 0;
//}