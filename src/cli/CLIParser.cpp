#include "CLIParser.h"
#include "ControllerFactory.h"

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
            ("home", po::value<int>(), "move to 0 position");
}

po::variables_map cli::CLIParser::get_variables_map(int argc, char *argv[]) {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    return vm;
}

po::variables_map cli::CLIParser::get_variables_map(const std::string &input) {
    po::variables_map vm;
    po::store(po::command_line_parser(tokenize(input))
                      .options(desc).run(), vm);
    po::notify(vm);
    return vm;
}

std::vector<std::string> cli::CLIParser::tokenize(const std::string &input) {
    typedef boost::escaped_list_separator<char> separator_type;
    separator_type separator("\\",    // The escape characters.
                             "= ",    // The separator characters.
                             "\"\'"); // The quote characters.

    // Tokenize the intput.
    boost::tokenizer<separator_type> tokens(input, separator);

    // Copy non-empty tokens from the tokenizer into the result.
    std::vector<std::string> result;
    copy_if(tokens.begin(), tokens.end(), std::back_inserter(result),
            !boost::bind(&std::string::empty, _1));
    return result;
}

template<typename InputIterator, typename OutputIterator, typename Predicate>
OutputIterator cli::CLIParser::copy_if(InputIterator first,
                                       InputIterator last,
                                       OutputIterator result,
                                       Predicate pred) {
    while (first != last) {
        if (pred(*first))
            *result++ = *first;
        ++first;
    }
    return result;
}

