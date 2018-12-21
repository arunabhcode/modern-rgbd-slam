// Copyright 2018 Arunabh Sharma

#include <fstream>
#include <iostream>
#include <string>

#include "nlohmann/json.hpp"

int main(int argc, char* argv[])
{
  if (argc != 2)
  {
    std::cout << "Expected command: <name of executable> <name of json>"
              << std::endl;
    return 0;
  }
  nlohmann::json j;

  std::ifstream in_file(argv[1], std::ios::in | std::ios::binary);
  if (!in_file.is_open())
  {
    std::cout << "Unable to open file" << std::endl;
    return 0;
  }

  in_file >> j;
  std::cout << "Value of pi = " << j["pi"] << std::endl;
  return 0;
}
