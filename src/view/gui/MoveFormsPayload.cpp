#include "MoveFormsPayload.h"

MoveFormsPayload::MoveFormsPayload(const QString &to, const QString &by) {
    if (!to.isEmpty()) {
        this->move_method = MoveMethod::TO;
        this->deg = atoi(to.toUtf8().constData());
    } else if (!by.isEmpty()) {
        this->move_method = MoveMethod::BY;
        this->deg = atoi(by.toUtf8().constData());
    }
}

