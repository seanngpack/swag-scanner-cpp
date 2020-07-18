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
            ("filter_test", "test filtering")
            ("name", po::value<std::string>(), "set name of scan")
            ("deg", po::value<int>(), "degrees")
            ("rot", po::value<int>(), "number of rotations")
            ("s_alpha", po::value<float>(), "smooth alpha")
            ("s_delta", po::value<int>(), "smooth delta")
            ("t_alpha", po::value<float>(), "smooth alpha")
            ("t_delta", po::value<int>(), "smooth delta")
            ("d_mag", po::value<int>(), "decimation filter magnitude")
            ("s_mag", po::value<int>(), "spatial filter magnitude")
            ("t_persis", po::value<int>(), "persistency index");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    return cli::ControllerFactory::create(vm);
}

