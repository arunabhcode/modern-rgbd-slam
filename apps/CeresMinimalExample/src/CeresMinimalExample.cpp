// Copyright 2018 Arunabh Sharma

#include "ceres/ceres.h"

struct CostFunctor
{
  template <typename T>
  bool operator()(const T* const x, T* residual) const
  {
    residual[0] = 25.0 - x[0] * x[0];
    return true;
  }
};

int main()
{
  double x               = 0.5;
  const double initial_x = x;

  // Build the problem.
  ceres::Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, NULL, &x);

  // Run the solver!
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x << " -> " << x << "\n";
  return 0;
}
