#include "FileHandler.h"

using namespace boost::filesystem;

file::FileHandler::FileHandler(std::string all_data_folder_path)
        : all_data_folder_path(check_input(all_data_folder_path) ? all_data_folder_path : nullptr),
          current_scan_folder_path(find_current_scan_folder(all_data_folder_path)) {

}

void file::FileHandler::set_folder_path(std::string path) {
    check_input(path);
    this->current_scan_folder_path = path;
}

std::string file::FileHandler::get_current_scan_folder() {
    return this->current_scan_folder_path;
}

std::string file::FileHandler::find_current_scan_folder(std::string folder) {
    check_input(folder);

    // if folder is empty let's start at 1.
    if (is_empty(folder)) {
        std::string name = folder + "/1";
        current_scan_folder_path = name;
        return name;
    }

    // make a vector to hold paths of all FOLDERS in the path
    std::vector<path> v;
    // don't include directories with '.' or without numbers in them.
    for (auto &&x : directory_iterator(folder)) {
        if (x.path().string().find('.') == std::string::npos &&
            x.path().string().find_first_of("0123456789") != std::string::npos) {
            v.push_back(x.path());
        }
    }

    // sort them using custom lambda function to order.
    std::sort(v.begin(), v.end(),
              [](auto &&path1, auto &&path2) {

                  std::string string1 = path1.string();
                  std::string string2 = path2.string();

                  // find the ending numbers of string1
                  size_t last_index = string1.find_last_not_of("0123456789");
                  std::string result1 = (string1.substr(last_index + 1));
                  // find the ending numbers of string 2
                  last_index = string2.find_last_not_of("0123456789");
                  std::string result2 = string2.substr(last_index + 1);

                  if (result1.length() == 0) {
                      return true;
                  } else if (result2.length() == 0) {
                      return true;
                  }
                  return (std::stoi(result1) < std::stoi(result2));
              }
    );


    // get the last item in list, convert to string, convert to int, then add 1
    int name_count = std::stoi(v.back().filename().string()) + 1;
    std::string name_count_str = std::to_string(name_count);

    std::string last_path = v.back().string();
    std::string to_replace = v.back().filename().string();
    std::string name = last_path.replace(last_path.find(to_replace),
                                         sizeof(name_count_str) - 1,
                                         name_count_str);
    return name;
}

bool file::FileHandler::check_input(std::string folder) {
    if (!is_directory(folder)) {
        throw std::invalid_argument("Error, the folder path you entered does not exist.");
    }
    return true;
}