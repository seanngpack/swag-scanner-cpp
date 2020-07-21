#include "Arduino.h"
#include <thread>

using namespace std::literals::chrono_literals;

arduino::Arduino::Arduino() {
    event_handler = new handler::ArduinoEventHandler();
    event_handler->connect_bluetooth();
}

void arduino::Arduino::rotate_table(int deg) {
    event_handler->rotate_table(deg);
}

void arduino::Arduino::rotate_to(int pos) {
    event_handler->rotate_to(pos);
}

arduino::Arduino::~Arduino() {
    delete event_handler;
}




