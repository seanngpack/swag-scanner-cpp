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

    static std::shared_ptr<spdlog::details::thread_pool> _tp = std::make_shared<spdlog::details::thread_pool>(51200, 4);
    static auto dist_sink = std::make_shared<spdlog::sinks::dist_sink_mt>(
            std::vector<spdlog::sink_ptr>({}));


    inline std::shared_ptr<spdlog::logger> setup_default_logger() {
        auto default_logger = spdlog::stdout_logger_mt("default_logger");
        return default_logger;
    }


    inline std::shared_ptr<spdlog::logger> setup_file_logger() {
        auto logger = std::make_shared<spdlog::logger>("backend_logger",
                                                       dist_sink);
        // [Oct 20 2020] some logging message
        logger->flush_on(spdlog::level::info);
//            logger->set_pattern("[%b %d %Y] %v");
        return logger;
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

    /**
     * Write an info level message. Write to both file and console. If the file sink hasn't been set up,
     * then only write to console.
     * @param message message you want to write.
     */
    inline void info(const std::string &message) {
        auto default_logger = spdlog::get("default_logger");
        auto file_logger = spdlog::get("backend_logger");
        if (file_logger != nullptr) {
            file_logger->info(message);
        }
        default_logger->info(message);
    }

    inline void debug(const std::string &message) {
        auto default_logger = spdlog::get("default_logger");
        auto file_logger = spdlog::get("backend_logger");
        if (file_logger != nullptr) {
            file_logger->debug(message);
        }
        default_logger->debug(message);
    }

}

#endif //SWAG_SCANNER_LOGGER_H
