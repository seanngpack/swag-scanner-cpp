/**
 * Test the wrapper on a main method to test async actions. Can't really unit test
 * this bad boy.
 */
#import "CoreBluetoothWrapper.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace std::literals::chrono_literals;

static bool finished = false;

void print_crap() {
    while (!finished) {
        std::cout << "printing thread: " << std::this_thread::get_id() << std::endl;
        std::this_thread::sleep_for(1s);
    }
}

void rotate(void *obj) {
    rotate_table(obj, 30);
}

void run_bt(void *obj) {
    std::cout << "bt thread: " << std::this_thread::get_id() << std::endl;

    start_bluetooth(obj);
}

int main() {
    std::cout << "main thread: " << std::this_thread::get_id() << std::endl;
    void *p = get_bluetooth_obj();
    run_bt(p);


    using namespace std::chrono_literals;

    std::this_thread::sleep_for(5s);
    std::cout << "cool" << std::endl;
    rotate(p);
    std::this_thread::sleep_for(3s);
    rotate(p);
    std::this_thread::sleep_for(3s);
    rotate(p);
    std::this_thread::sleep_for(10s);
    rotate(p);
    std::this_thread::sleep_for(10s);


    return 0;
}