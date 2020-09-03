#include "FormsPayload.h"

FormsPayload::FormsPayload(const QString &name, const QString &deg, const QString &rot) {
    this->name = name.toUtf8().constData();
    // i could use stoi with error handling, but this is faster and I consider 0 to be an error anyways,
    this->deg = atoi(deg.toUtf8().constData());
    this->rot = atoi(rot.toUtf8().constData());
}
