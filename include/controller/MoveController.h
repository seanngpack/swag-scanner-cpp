#ifndef SWAG_SCANNER_MOVECONTROLLER_H
#define SWAG_SCANNER_MOVECONTROLLER_H

#include "IController.h"
#include "Arduino.h"

namespace controller {
    /**
     * This controller allows for manual rotation of the table using commands from the commandline.
     */
    class MoveController : public IController {
    public:

        MoveController(std::shared_ptr<arduino::Arduino> arduino);

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
         * Set the table either to move TO a position, or BY a degree amount.
         *
         * Note: Using string comparison. Not ideal.
         * @param input either "to" or "by" command.
         */
        void set_move_method(const std::string &input);


    private:
        std::shared_ptr<arduino::Arduino> arduino;
        std::string move_method;
        int deg = 0;


    };
}

#endif //SWAG_SCANNER_MOVECONTROLLER_H
