#include "ProcessingController.h"
#include "Visualizer.h"
#include "ProcessingModel.h"
#include <memory>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

controller::ProcessingController::ProcessingController(std::shared_ptr<model::ProcessingModel> model) :
        model(std::move(model)) {}

void controller::ProcessingController::run() {
    model->transform_clouds_to_world();
    model->filter();
    model->register_clouds();
}
