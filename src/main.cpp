#include "CLIParser.h"
#include "IController.h"
#include "ControllerManager.h"
#include "ControllerManagerCache.h"
#include "IFileHandler.h"
#include "SwagGUI.h"
#include "Logger.h"
#include <spdlog/logger.h>
#include <boost/program_options.hpp>
#include <QApplication>


int main(int argc, char *argv[]) {
    file::IFileHandler::check_program_folder();

    auto default_logger = logger::setup_default_logger();
    std::unique_ptr<cli::CLIParser> cli_parser = std::make_unique<cli::CLIParser>();
    boost::program_options::variables_map vm = cli_parser->get_variables_map(argc, argv);

    if (vm.count("gui")) {
        auto file_logger = logger::setup_file_logger();
        // this is sloppy, but i need it to initialize the logger. I should make a function that deletes the generated file.
        logger::set_file_logger_location(file::IFileHandler::swag_scanner_path.string() + "/settings/log_init.txt");
        spdlog::register_logger(file_logger);
        spdlog::set_level(spdlog::level::level_enum::debug);
        file_logger->flush_on(spdlog::level::info);

        QApplication app(argc, argv);
        controller::ControllerManager manager;
        std::shared_ptr<SwagGUI> gui = manager.get_gui();
        gui->show();
        return app.exec();
    } else {
        controller::ControllerManager manager;
        std::shared_ptr<controller::IController> controller = manager.get_controller(vm);
        controller->run();
        return 0;
    }
    // pretty sure this does nothing
    spdlog::shutdown();
}
