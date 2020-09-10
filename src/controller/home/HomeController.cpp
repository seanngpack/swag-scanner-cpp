#include "IFileHandler.h"
#include "HomeController.h"

using json = nlohmann::json;

void controller::HomeController::run() {
    json settings_json = file::IFileHandler::load_swag_scanner_info_json();
    settings_json["current_position"] = 0;
    file::IFileHandler::write_swag_scanner_info_json(settings_json);
}
