#ifndef SWAG_SCANNER_ARDUINO_H
#define SWAG_SCANNER_ARDUINO_H

#include <iostream>

namespace arduino {
    class Arduino {
    public:

        Arduino();

        /**
         * Rotate the scanner table CCW.
         * @param deg number of degrees to rotate.
         */
        void rotate_table(int deg);

        /**
         * Set the isRotating parameter to either true or false;
         * @param in the bool
         */
        void setIsRotating(bool in);

        /**
         * Set the isConnected field to the input.
         * @param in true if the bluetooth device and services are connected.
         */
        void setIsConnected(bool in);


    private:
        bool isConnected = false;
        bool isRotating = false;
        void *bluetooth_object;
        std::string UART_SERVICE_UUID = "5ffba521-2363-41da-92f5-46adc56b2d37";
        std::string ROTATE_TABLE_CHAR_UUID = "5ffba522-2363-41da-92f5-46adc56b2d37";
        std::string TABLE_POSITION_CHAR_UUID = "5ffba523-2363-41da-92f5-46adc56b2d37";
        std::string IS_TABLE_ROTATING_CHAR_UUID = "5ffba524-2363-41da-92f5-46adc56b2d37";

        /**
         * Poll until the connection and subscription is complete.
         * Blocks the thread the arduino is running on until connection and subscription to
         * bluetooth device is secured.
         */
        void wait_for_connection();
    };
}

#endif //SWAG_SCANNER_ARDUINO_H
