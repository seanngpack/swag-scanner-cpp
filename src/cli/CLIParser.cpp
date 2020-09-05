#include "CLIParser.h"
#include <boost/bind.hpp>

namespace po = boost::program_options;

cli::CLIParser::CLIParser() {
//    desc = po::options_description
//    desc("SwagScanner options");
    desc.add_options()
            ("help, h", "lol no help here")
            // main commands
            ("scan", "scan object")
            ("calibrate", "calibrate scanner")
            ("process", "process scanned data")
            ("filter_test", "test filtering")
            ("move", "move calibration bed")
            ("set_home", "move calibration bed")

            // gui
            ("gui", "start gui application")

            // arguments for main commands
            ("name", po::value<std::string>(), "set name of scan")
            ("deg", po::value<int>(), "degrees")
            ("rot", po::value<int>(), "number of rotations")
            ("s_alpha", po::value<float>(), "smooth alpha")
            ("s_delta", po::value<int>(), "smooth delta")
            ("d_mag", po::value<int>(), "decimation filter magnitude")
            ("s_mag", po::value<int>(), "spatial filter magnitude")
            ("to", po::value<int>(), "move to a position")
            ("by", po::value<int>(), "move by degrees")
            ("home", "move to 0 position");
}

po::variables_map cli::CLIParser::get_variables_map(int argc, char *argv[]) {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    return vm;
}

