#ifndef SWAG_SCANNER_ARDUINOEVENTHANDLER_H
#define SWAG_SCANNER_ARDUINOEVENTHANDLER_H

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

        void rotate_by(int degs);

        void rotate_to(int target);

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

        /**
         * If x is less than y, return x.
         * If x is greater than y, return -y.
         * @param x
         * @param y
         * @return
         */
        int get_least(int x, int y);

    };
}

#endif //SWAG_SCANNER_ARDUINOEVENTHANDLER_H
