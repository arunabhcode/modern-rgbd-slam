// Copyright 2018 Arunabh Sharma

#include <iostream>

#include "sophus/so3.hpp"

int main()
{
  Sophus::SO3<float>::Point p_inst(1.0f, 2.0f, 3.0f);
  std::cout << "Sophus SO3 printing \n" << p_inst << std::endl;
  return 0;
}
