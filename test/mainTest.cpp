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

void rotate(void* obj) {
    rotate_table(obj, 30);
}

void run_bt(void* obj) {
    std::cout << "bt thread: " << std::this_thread::get_id() << std::endl;

    start_bt(obj);
}

int main() {
    std::cout << "main thread: " << std::this_thread::get_id() << std::endl;
    void *p = get_wrapper_object();
    std::thread worker(run_bt, p);
//    worker.detach();



    using namespace std::chrono_literals;

    std::this_thread::sleep_for(5s);
    std::cout << "cool" <<std::endl;
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