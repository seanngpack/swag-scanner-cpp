#include "CLIClient.h"
#include "ControllerFactory.h"
#include <boost/program_options.hpp>

namespace po = boost::program_options;

cli::CLIClient::CLIClient() {}

std::unique_ptr<controller::IController> cli::CLIClient::get_controller(int argc, char *argv[]) {
    po::options_description desc("SwagScanner options");
    desc.add_options()
            ("help, h", "lol no help here")
            ("scan", "scan object")
            ("calibrate", "calibrate scanner")
            ("process", "process scanned data")
            ("name", po::value<std::string>(), "set name of scan")
            ("deg", po::value<int>(), "degrees")
            ("rot", po::value<int>(), "number of rotations");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    return cli::ControllerFactory::create(vm);

}

