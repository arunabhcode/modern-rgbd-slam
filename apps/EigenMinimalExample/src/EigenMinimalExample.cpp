// Copyright 2019 Arunabh Sharma

#include <iostream>

#include "Eigen/Core"

int main(int argc, char** argv)
{
    Eigen::Matrix3f identity = Eigen::Matrix3f::Identity(3, 3);
    std::cout << "Identity 3x3 Matrix = \n" << identity << std::endl;
    return 0;
}
