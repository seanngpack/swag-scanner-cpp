#ifndef SWAG_SCANNER_HOMECONTROLLER_H
#define SWAG_SCANNER_HOMECONTROLLER_H

#include "IController.h"

namespace controller {

    /**
     * This controller handles homing of the scanner.
     */
    class HomeController : public IController {
    public:
        HomeController() = default;

        /**
         * Go to home position.
         */
        void run() override;
    };
}

#endif //SWAG_SCANNER_HOMECONTROLLER_H
