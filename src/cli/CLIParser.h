#ifndef SWAG_SCANNER_CLIPARSER_H
#define SWAG_SCANNER_CLIPARSER_H


#include <boost/program_options.hpp>

namespace cli {
    /**
     * This class keeps track of shared resources and uses the manager to create
     * new controllers;
     */
    class CLIParser {
    public:

        CLIParser();

        boost::program_options::variables_map get_variables_map(int argc, char *argv[]);


    private:
        boost::program_options::options_description desc;





    };

};

#endif //SWAG_SCANNER_CLIPARSER_H
