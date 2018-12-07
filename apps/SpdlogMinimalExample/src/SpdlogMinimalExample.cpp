// Copyright 2018 Arunabh Sharma

#include <iostream>
#include <string>

#include "spdlog/spdlog.h"

int main()
{
    spdlog::set_level(spdlog::level::trace);  // Set specific logger's log level
    spdlog::trace("This will print trace level message");
    spdlog::info("This will print info level message");
    spdlog::debug("This will print debug level message");
    spdlog::warn("This will print warn level message");
    spdlog::critical("This will print critical level message");
    spdlog::error("This will print error level message");
    return 0;
}
