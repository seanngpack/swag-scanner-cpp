#include "ProcessingController.h"
#include "Visualizer.h"
#include "ProcessingModel.h"
#include "Logger.h"
#include <memory>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

controller::ProcessingController::ProcessingController(std::shared_ptr<model::ProcessingModel> model) :
        model(std::move(model)) {}

void controller::ProcessingController::run() {
    logger::info("starting processing");
    logger::info("TRANSFORMING-------");
    model->transform_clouds_to_world();
    logger::info("FINISHED TRANSFORMING-------");
    logger::info("FILTERING-------");
    model->filter();
    logger::info("FINISHED FILTERING-------");
    logger::info("REGISTERING-------");
    model->register_clouds();
    logger::info("FINISHED REGISTERING-------");
}
