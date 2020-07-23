#include "IFileHandler.h"
#include "HomeController.h"

using json = nlohmann::json;

void controller::HomeController::run() {
    json settings_json = file::IFileHandler::load_settings_json();
    settings_json["current_position"] = 0;
    file::IFileHandler::write_settings_json(settings_json);
}
