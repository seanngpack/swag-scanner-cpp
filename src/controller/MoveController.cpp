#include <MoveController.h>

controller::MoveController::MoveController(std::shared_ptr<arduino::Arduino> arduino) :
        arduino(std::move(arduino)) {}

void controller::MoveController::run() {
    arduino->rotate_table(deg);
}

void controller::MoveController::set_deg(int degs) {
    this->deg = degs;
}

