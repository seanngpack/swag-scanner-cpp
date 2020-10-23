// https://github.com/gabime/spdlog/wiki/How-to-use-spdlog-in-DLLs
#ifndef SWAG_SCANNER_LOGGER_H
#define SWAG_SCANNER_LOGGER_H


#include <memory>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/dist_sink.h>

namespace logger {

//    static auto default_file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("");
    static std::shared_ptr<spdlog::details::thread_pool> _tp = std::make_shared<spdlog::details::thread_pool>(51200, 4);
    static auto dist_sink = std::make_shared<spdlog::sinks::dist_sink_mt>(
            std::vector<spdlog::sink_ptr>({}));


    inline std::shared_ptr<spdlog::logger> setup_logger() {
        std::vector<spdlog::sink_ptr> sinks;
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_st>());
        auto logger = spdlog::get("basic_logger");
        {
            logger = std::make_shared<spdlog::logger>("basic_logger",
                                                      std::begin(sinks),
                                                      std::end(sinks));
//            spdlog::register_logger(logger);
            return logger;
        }
    }

    inline std::shared_ptr<spdlog::logger> setup_file_logger() {
        {
            auto logger = std::make_shared<spdlog::logger>("backend_logger",
                                                           dist_sink);
            // [Oct 20 2020] some logging message
            logger->flush_on(spdlog::level::info);
//            logger->set_pattern("[%b %d %Y] %v");
            return logger;
        }
    }

    inline std::shared_ptr<spdlog::logger> get_file_logger() {
        return spdlog::get("backend_logger");
    }

    /**
     * Change location of output file of the file logger.
     * Make sure to put the entire path + file.
     * Example: "/user/files/swagscanner/scans/scan1/info/basic_logger.txt"
     * Note: may exist a better way to do this, kinda inefficient to keep spawning new sinks.
     *
     * @param path full path to the new log text file.
     */
    inline void set_file_logger_location(const std::string &path) {
        auto new_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path);
        dist_sink->set_sinks(std::vector<spdlog::sink_ptr>({new_sink}));
    }

    inline void file_logger_write(const std::string &message) {
        auto logger = spdlog::get("backend_logger");
        if (logger == nullptr) {
            return;
        }
        logger->info(message);
    }

}

#endif //SWAG_SCANNER_LOGGER_H
