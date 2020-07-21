#ifndef SWAG_SCANNER_MOVECONTROLLER_H
#define SWAG_SCANNER_MOVECONTROLLER_H

#include "IController.h"
#include "Arduino.h"

namespace controller {
    /**
     * This controller takes commands from c
     */
    class MoveController : public IController {
    public:

        MoveController(std::shared_ptr<arduino::Arduino> arduino);

        void run() override;

        void set_deg(int deg);

        void set_move_method(const std::string &input);

        std::string move_method;

    private:
        std::shared_ptr<arduino::Arduino> arduino;
        int deg = 0;



    };
}

#endif //SWAG_SCANNER_MOVECONTROLLER_H
