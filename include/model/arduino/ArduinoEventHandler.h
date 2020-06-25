#ifndef SWAG_SCANNER_ARDUINOEVENTHANDLER_H
#define SWAG_SCANNER_ARDUINOEVENTHANDLER_H

#include "CoreBluetoothWrapper.h"
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <functional>

namespace handler {

    class ArduinoEventHandler {
    public:

        ArduinoEventHandler();

        void rotate_table(int degs);

        void connect_bluetooth();

        void set_is_bt_connected(bool is_connected);

        void set_is_table_rotating(bool is_rotating);

        ~ArduinoEventHandler();

        std::mutex bt_mutex;
        std::mutex table_mutex;
        std::condition_variable bt_cv;
        std::condition_variable table_cv;

    private:
        void *bluetooth_object;
        bool is_bt_connected;
        bool is_table_rotating;

    };
}

#endif //SWAG_SCANNER_ARDUINOEVENTHANDLER_H
