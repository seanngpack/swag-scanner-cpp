#ifndef SWAG_SCANNER_ICONTROLLER_H
#define SWAG_SCANNER_ICONTROLLER_H

namespace controller {
    /**
     * Represents an abstract base for controllers.
     */
    class IController {
    public:
        /**
         * Run the controller
         */
        virtual void run();

        virtual ~IController() {}
    };
}

#endif //SWAG_SCANNER_ICONTROLLER_H
