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

//--filter_test --d_mag 1 --s_mag 5 --s_delta 40 --s_alpha .5