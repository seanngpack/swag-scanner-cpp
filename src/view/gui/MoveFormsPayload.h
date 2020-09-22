#ifndef SWAG_SCANNER_MOVEFORMSPAYLOAD_H
#define SWAG_SCANNER_MOVEFORMSPAYLOAD_H

#include "IFormsPayload.h"
#include "MoveMethod.h"
#include <string>
#include <QString>

/**
 * Represents an object containing information from the move forms.
 */
struct MoveFormsPayload : public IFormsPayload {
    /**
     * This constructor creates a new MoveFormsPayload object.
     * If both the to and by forms are filled, then it will use the to form.
     *
     * @param method either to or by.
     * @param deg the number of degrees.
     */
    explicit MoveFormsPayload(const MoveMethod &method, int deg);

    MoveMethod move_method;
    int deg;

};

#endif //SWAG_SCANNER_MOVEFORMSPAYLOAD_H
