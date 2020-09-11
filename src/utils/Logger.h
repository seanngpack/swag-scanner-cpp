// https://github.com/gabime/spdlog/wiki/How-to-use-spdlog-in-DLLs
#ifndef SWAG_SCANNER_LOGGER_H
#define SWAG_SCANNER_LOGGER_H

#include <memory>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace logger {
    static const std::string logger_name = "swag_logger";

    std::shared_ptr<spdlog::logger> setup_logger(const std::vector<spdlog::sink_ptr> &sinks) {
        auto logger = spdlog::get(logger_name);
        {
            logger = std::make_shared<spdlog::logger>(logger_name,
                                                      std::begin(sinks),
                                                      std::end(sinks));
            spdlog::register_logger(logger);

            return logger;
        }
    }
}

#endif //SWAG_SCANNER_LOGGER_H
