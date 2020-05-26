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

void run_bt() {
    std::cout << "bt thread: " << std::this_thread::get_id() << std::endl;
    void *p = get_wrapper_object();
}

int main() {
    std::cout << "main thread: " << std::this_thread::get_id() << std::endl;
    std::thread worker(run_bt);
    worker.detach();
    print_crap();


    return 0;
}