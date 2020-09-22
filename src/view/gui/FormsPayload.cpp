#include "FormsPayload.h"

FormsPayload::FormsPayload(const QString &name, int angle, int rotations) :
        angle(angle), rotations(rotations) {
    this->name = name.toUtf8().constData();
}
