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
     * @param to to field.
     * @param by by field.
     */
    explicit MoveFormsPayload(const QString &to, const QString &by);

    MoveMethod move_method;
    int deg;

};

#endif //SWAG_SCANNER_MOVEFORMSPAYLOAD_H
