#include "ProcessingController.h"
#include "Visualizer.h"
#include "ProcessingModel.h"
#include <memory>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

controller::ProcessingController::ProcessingController(std::shared_ptr<model::ProcessingModel> model) :
        model(std::move(model)) {}

void controller::ProcessingController::run() {
    std::cout << "transforming.." << std::endl;
    model->transform_clouds_to_world();
    std::cout << "filtering.." << std::endl;
    model->filter();
    std::cout << "registering.." << std::endl;
    model->register_clouds();
}
