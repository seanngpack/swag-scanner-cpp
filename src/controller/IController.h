#ifndef SWAG_SCANNER_ICONTROLLER_H
#define SWAG_SCANNER_ICONTROLLER_H

namespace controller {
    class IController {
    public:
        /**
         * Run the controller
         */
        virtual void run() = 0;

        virtual ~IController() {}
    };
}

#endif //SWAG_SCANNER_ICONTROLLER_H
