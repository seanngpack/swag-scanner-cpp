#ifndef SWAG_SCANNER_MOVECONTROLLER_H
#define SWAG_SCANNER_MOVECONTROLLER_H

#include "IController.h"
#include "MoveMethod.h"
#include <string>

namespace arduino {
    class Arduino;
}

namespace controller {
    /**
     * This controller allows for manual rotation of the table using commands from the commandline.
     */
    class MoveController : public IController {
    public:

        explicit MoveController(std::shared_ptr<arduino::Arduino> arduino);

        /**
         * Move table to position, or by given amount.
         */
        void run() override;

        /**
         * Set the degrees.
         * @param deg degrees.
         */
        void set_deg(int deg);

        /**
         * Set the controller to move the table either move TO a position, or BY a degree amount.
         *
         * @param input either "to" or "by" command.
         */
        void set_move_method(const MoveMethod &move_method);


    protected:
        std::shared_ptr<arduino::Arduino> arduino;
        MoveMethod move_method;
        int deg = 0;
    };
}

#endif //SWAG_SCANNER_MOVECONTROLLER_H
