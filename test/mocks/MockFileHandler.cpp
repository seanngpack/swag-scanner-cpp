#include <FileHandler.h>
#include "gmock/gmock.h"

class MockFileHandler : public file::FileHandler {
    public:

    MockFileHandler(std::string all_data_folder_path) : file::FileHandler(all_data_folder_path) {}



};