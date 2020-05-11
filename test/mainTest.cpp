#import "CoreBluetoothWrapper.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace std::literals::chrono_literals;

static bool finished = false;

void print_crap() {
    while (!finished) {
//        std::cout << "working in background thread?: " << std::this_thread::get_id() << std::endl;
        std::this_thread::sleep_for(1s);
    }

}

int main() {
    std::thread worker(print_crap);
    void *p = get_wrapper_object();

    // these never get called because my bluetooth object is blocking main thread
    std::cout << "memory address of object: ";
    std::cout << p << std::endl;
    print_central_address(p);
    std::cout << std::this_thread::get_id();



    return 0;
}