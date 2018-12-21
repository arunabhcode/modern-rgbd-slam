// Copyright 2018 Arunabh Sharma

#include <iostream>
#include <string>

#include "librealsense2/rs.hpp"

int main()
{
  rs2::context ctx;

  std::vector<rs2::sensor> list = ctx.query_all_sensors();

  // std::string my_type = typeid(list).name();
  // std::cout << system(("echo " + my_type + " | c++filt -t").c_str()) <<
  // std::endl;

  if (list.size() > 0)
  {
    std::cout << "Device : " << list[0] << std::endl;
  }
  else
  {
    std::cout << "No devices connected" << std::endl;
  }
  return 0;
}
