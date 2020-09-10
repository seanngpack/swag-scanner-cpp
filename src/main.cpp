#include "CLIParser.h"
#include "IController.h"
#include "factory/ControllerFactory.h"
#include "factory/ControllerFactoryCache.h"
#include "SwagGUI.h"
#include <boost/program_options.hpp>
#include <iostream>
#include <QApplication>


int main(int argc, char *argv[]) {
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
