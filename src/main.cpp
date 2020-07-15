#include <iostream>
#include "CLIClient.h"
#include "IController.h"


int main(int argc, char* argv[]) {
    std::unique_ptr<cli::CLIClient> cli_client = std::make_unique<cli::CLIClient>();
    std::unique_ptr<controller::IController> controller = cli_client->get_controller(argc, argv);
    controller->run();
    return 0;
}
