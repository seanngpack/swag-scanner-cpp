#ifndef SWAG_SCANNER_ARDUINO_H
#define SWAG_SCANNER_ARDUINO_H

#include <iostream>
#include <memory>
#include "feeling-blue/feeling-blue.h"


namespace arduino {
    class Arduino {
    public:

        Arduino();

        /**
         * Rotate the scanner table by input.
         * + num for CCW
         * - num for CW
         * @param deg number of degrees to rotate.
         */
        void rotate_by(int deg);

        /**
         * Rotate to given target.
         * @param target position to rotate to.
         */
        void rotate_to(int target);


        ~Arduino() = default;


    private:
        int current_pos;
        std::unique_ptr<bluetooth::Central> central_manager;
        std::shared_ptr<bluetooth::Peripheral> arduino;
        std::shared_ptr<bluetooth::Service> service;
        std::shared_ptr<bluetooth::Characteristic> rotate_char;
        std::shared_ptr<bluetooth::Characteristic> table_pos_char;
        std::shared_ptr<bluetooth::Characteristic> is_table_rot_char;
        std::string DEVICE_NAME;
        std::string UART_SERVICE_UUID = "5ffba521-2363-41da-92f5-46adc56b2d37";
        std::string ROTATE_TABLE_CHAR_UUID = "5ffba522-2363-41da-92f5-46adc56b2d37";
        std::string TABLE_POSITION_CHAR_UUID = "5ffba523-2363-41da-92f5-46adc56b2d37";
        std::string IS_TABLE_ROTATING_CHAR_UUID = "5ffba524-2363-41da-92f5-46adc56b2d37";

        /**
         * Updates the current position in the settings.json file.
         * TODO: this is very slow because I'm reading the file, then writing to it.
         */
        void update_current_pos();

        int get_least(int x, int y);

    };
}

#endif //SWAG_SCANNER_ARDUINO_H
