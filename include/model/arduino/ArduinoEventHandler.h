#ifndef SWAG_SCANNER_ARDUINOEVENTHANDLER_H
#define SWAG_SCANNER_ARDUINOEVENTHANDLER_H

#include "CoreBluetoothWrapper.h" // TODO: see if I can move this to source file
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <functional>

namespace handler {

    class ArduinoEventHandler {
    public:

        /**
         * On initialization, will search for settings.json and set the current position
         * from that file.
         * Might be bad design because arduino is kinda taking responsibility from the
         * filehandler class.
         */
        ArduinoEventHandler();

        void rotate_table(int degs);

        void rotate_to(int pos);

        void connect_bluetooth();

        void set_is_bt_connected(bool is_connected);

        void set_is_table_rotating(bool is_rotating);

        /**
         * Updates the current position in the settings.json file.
         * TODO: this is very slow because I'm reading the file, then writing to it.
         */
        void update_current_pos();

        ~ArduinoEventHandler();

        std::mutex bt_mutex;
        std::mutex table_mutex;
        std::condition_variable bt_cv;
        std::condition_variable table_cv;

    private:
        int current_pos;
        void *bluetooth_object;
        bool is_bt_connected;
        bool is_table_rotating;

    };
}

#endif //SWAG_SCANNER_ARDUINOEVENTHANDLER_H
