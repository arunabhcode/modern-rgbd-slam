// Copyright 2018 Arunabh Sharma

#include "spdlog/spdlog.h"

int main()
{
    spdlog::set_level(spdlog::level::trace);  // Set specific logger's log level

    spdlog::trace("This will print trace level message");
    // To use to tell the user something or general landmark output
    spdlog::info("This will print info level message");
    // To debug
    spdlog::debug("This will print debug level message");
    // To warn the user in case of some non-expected/borderline value
    spdlog::warn("This will print warn level message");
    // To log some critical assertion failure
    spdlog::critical("This will print critical level message");
    // In case of try catching
    spdlog::error("This will print error level message");
    return 0;
}
