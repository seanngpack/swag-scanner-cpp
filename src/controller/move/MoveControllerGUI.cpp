#include "MoveControllerGUI.h"
#include "Arduino.h"
#include "SwagGUI.h"
#include "MoveMethod.h"
#include "MoveFormsPayload.h"

controller::MoveControllerGUI::MoveControllerGUI(std::shared_ptr<arduino::Arduino> arduino,
                                                 std::shared_ptr<SwagGUI> gui) :
        MoveController(std::move(arduino)),
        IControllerGUI(std::move(gui)) {}

void controller::MoveControllerGUI::run() {
    if (move_method == MoveMethod::TO) {
        emit update_console("moving to position: " + std::to_string(deg));
        arduino->rotate_to(deg);
        emit update_console("done moving");
    } else if (move_method == MoveMethod::BY) {
        emit update_console("moving by: " + std::to_string(deg) + " degrees");
        arduino->rotate_by(deg);
        emit update_console("done moving");
    }
}

void controller::MoveControllerGUI::update(const IFormsPayload &payload) {
    const auto &p = dynamic_cast<const MoveFormsPayload &>(payload);
    set_move_method(p.move_method);
    set_deg(p.deg);
}
