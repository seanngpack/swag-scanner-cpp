#ifndef SWAG_SCANNER_MOVECONTROLLER_H
#define SWAG_SCANNER_MOVECONTROLLER_H

#include "IController.h"
#include "Arduino.h"

namespace controller {
    /**
     * This controller takes commands from c
     */
    class MoveController : IController {
    public:

        MoveController(std::shared_ptr<arduino::Arduino> arduino);

        void run() override;

        void set_deg(int deg);

    private:
        std::shared_ptr<arduino::Arduino> arduino;
        int deg = 0;

    };
}

#endif //SWAG_SCANNER_MOVECONTROLLER_H
