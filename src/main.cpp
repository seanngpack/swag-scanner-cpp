#include <iostream>
#include "CLIParser.h"
#include "IController.h"
#include "ControllerFactory.h"
#include <boost/program_options.hpp>


int main(int argc, char* argv[]) {
    cli::ControllerFactory factory;
    std::unique_ptr<cli::CLIParser> cli_parser = std::make_unique<cli::CLIParser>();
    boost::program_options::variables_map vm = cli_parser->get_variables_map(argc, argv);
    std::unique_ptr<controller::IController> controller = factory.create(vm);
    controller->run();
    return 0;
}

#include <QApplication>
#include "IControllerGUI.h"
#include "SwagGUI.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    auto g = std::make_shared<controller::IControllerGUI>();
    SwagGUI gui(g);
    gui.show();

    return app.exec();
}



//--filter_test --d_mag 2 --s_mag 2 --s_alpha .5 --s_delta 10
//--filter_test --d_mag 1 --s_mag 2 --s_alpha .5 --s_delta 20
//--filter_test --d_mag 1 --s_mag 5 --s_alpha .45 --s_delta 5