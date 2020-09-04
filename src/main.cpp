#include "CLIParser.h"
#include "IController.h"
#include "ControllerFactory.h"
#include "ControllerFactoryCache.h"
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
        factory.get_gui_controller("calibrate");
//        SwagGUI gui;
//        gui.show();

        return app.exec();
    } else {
        controller::ControllerFactory factory;
        std::shared_ptr<controller::IController> controller = factory.get_controller(vm);
        controller->run();
        return 0;
    }


}

//#include <QApplication>
//#include "IControllerGUI.h"
//#include "SwagGUI.h"
//
//int main(int argc, char *argv[]) {
//    QApplication app(argc, argv);
//    auto g = std::make_shared<controller::IControllerGUI>();
//    SwagGUI gui(g);
//    gui.show();
//
//    return app.exec();
//}



//--filter_test --d_mag 2 --s_mag 2 --s_alpha .5 --s_delta 10
//--filter_test --d_mag 1 --s_mag 2 --s_alpha .5 --s_delta 20
//--filter_test --d_mag 1 --s_mag 5 --s_alpha .45 --s_delta 5