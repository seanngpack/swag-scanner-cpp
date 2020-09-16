#include "CLIParser.h"
#include "IController.h"
#include "ControllerFactory.h"
#include "ControllerFactoryCache.h"
#include "IFileHandler.h"
#include "SwagGUI.h"
#include "Logger.h"
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <boost/program_options.hpp>
#include <iostream>ardui
#include <QApplication>


int main(int argc, char *argv[]) {
    file::IFileHandler::check_program_folder();

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_st>());
    auto logger = logger::setup_logger(sinks);
    spdlog::set_level(spdlog::level::level_enum::debug);

    std::unique_ptr<cli::CLIParser> cli_parser = std::make_unique<cli::CLIParser>();
    boost::program_options::variables_map vm = cli_parser->get_variables_map(argc, argv);

    if (vm.count("gui")) {
        QApplication app(argc, argv);
        controller::ControllerFactory factory;
        std::shared_ptr<SwagGUI> gui = factory.get_gui();
        gui->show();
        return app.exec();
    } else {
        controller::ControllerFactory factory;
        std::shared_ptr<controller::IController> controller = factory.get_controller(vm);
        controller->run();
        return 0;
    }
}
