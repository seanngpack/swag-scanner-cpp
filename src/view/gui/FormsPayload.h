#ifndef SWAG_SCANNER_FORMSPAYLOAD_H
#define SWAG_SCANNER_FORMSPAYLOAD_H

#include <string>
#include <QString>

/**
 * Represents an object containing information from the forms.
 */
struct FormsPayload {
    /**
     * This constructor creates a new FormsPayload object. Does conversions from QString to std::string and int.
     * @param name name field.
     * @param deg degree field.
     * @param rot rotation field.
     */
    explicit FormsPayload(const QString &name, const QString &deg = QString(""), const QString &rot = QString(""));

    std::string name;
    int deg;
    int rot;
};

#endif //SWAG_SCANNER_FORMSPAYLOAD_H
