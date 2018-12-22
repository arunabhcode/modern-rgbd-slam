// Copyright 2018 Arunabh Sharma

#include <iostream>
#include <string>

#include "cxxopts.hpp"

int main(int argc, char* argv[])
{
  if (argc != 3)
  {
    std::cout
        << "Expected command: <app name> -o <message to print onto console>"
        << std::endl;
    return 0;
  }
  cxxopts::Options options("demo cxxopts usage", "demos cxxopts library usage");
  options.add_options()(
      "o,output", "Output message for console", cxxopts::value<std::string>());

  cxxopts::ParseResult result = options.parse(argc, argv);

  std::cout << "Argument Message: " << result["o"].as<std::string>()
            << std::endl;

  return 0;
}
