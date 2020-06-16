#ifndef SWAG_SCANNER_ARDUINO_H
#define SWAG_SCANNER_ARDUINO_H

#include <iostream>
#include "ArduinoEventHandler.h"

namespace arduino {
    class Arduino {
    public:

        Arduino();

        /**
         * Rotate the scanner table CCW.
         * @param deg number of degrees to rotate.
         */
        void rotate_table(int deg);


    private:
        handler::ArduinoEventHandler *event_handler;
        std::string UART_SERVICE_UUID = "5ffba521-2363-41da-92f5-46adc56b2d37";
        std::string ROTATE_TABLE_CHAR_UUID = "5ffba522-2363-41da-92f5-46adc56b2d37";
        std::string TABLE_POSITION_CHAR_UUID = "5ffba523-2363-41da-92f5-46adc56b2d37";
        std::string IS_TABLE_ROTATING_CHAR_UUID = "5ffba524-2363-41da-92f5-46adc56b2d37";

    };
}

#endif //SWAG_SCANNER_ARDUINO_H
