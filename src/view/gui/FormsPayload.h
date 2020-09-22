#ifndef SWAG_SCANNER_FORMSPAYLOAD_H
#define SWAG_SCANNER_FORMSPAYLOAD_H

#include "IFormsPayload.h"
#include <string>
#include <QString>

/**
 * Represents an object containing information from the forms.
 */
struct FormsPayload : public IFormsPayload {
    /**
     * This constructor creates a new FormsPayload object. Does conversions from QString to std::string and int.
     * @param name name field.
     * @param deg degree field.
     * @param rot rotation field.
     */
    explicit FormsPayload(const QString &name, int angle, int rotations);

    std::string name;
    int angle;
    int rotations;
};

#endif //SWAG_SCANNER_FORMSPAYLOAD_H
