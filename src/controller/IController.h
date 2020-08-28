#ifndef SWAG_SCANNER_ICONTROLLER_H
#define SWAG_SCANNER_ICONTROLLER_H

#include <iostream>

namespace controller {
    class IController {
    public:
        /**
         * Run the controller
         */
        virtual void run() = 0;

        virtual ~IController() {
            std::cout << "calling IController destructor \n";
        }
    };
}

#endif //SWAG_SCANNER_ICONTROLLER_H
