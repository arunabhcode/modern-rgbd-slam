// Copyright 2019 Arunabh Sharma

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"

class VertexParams : public g2o::BaseVertex<2, Eigen::Vector2d>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexParams()
  {
  }
  virtual bool read(std::istream& /*is*/)
  {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }

  virtual bool write(std::ostream& /*os*/) const
  {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }

  virtual void setToOriginImpl()
  {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
  }

  virtual void oplusImpl(const double* update)
  {
    Eigen::Vector2d::ConstMapType v(update);
    _estimate += v;
  }
};

class EdgePointOnCurve : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePointOnCurve()
  {
  }
  virtual bool read(std::istream& /*is*/)
  {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }
  virtual bool write(std::ostream& /*os*/) const
  {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }
  void computeError()
  {
    const VertexParams* params = static_cast<const VertexParams*>(vertex(0));
    const double& a            = params->estimate()(0);
    const double& b            = params->estimate()(1);
    double fval                = a - std::pow(measurement()(0), b);
    _error(0)                  = fval - measurement()(1);
  }
};

int main()
{
  int num_points          = 50;
  Eigen::Vector2d* points = new Eigen::Vector2d[num_points];
  for (int i = 0; i < num_points; ++i)
  {
    double x = g2o::Sampler::uniformRand(0, 10);
    double y = 25 - x * x;
    // add Gaussian noise
    y += g2o::Sampler::gaussRand(0, 0.02);
    points[i].x() = x;
    points[i].y() = y;
  }

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>> BlockSolver;
  typedef g2o::LinearSolverDense<BlockSolver::PoseMatrixType> LinearSolver;

  // setup the solver
  g2o::SparseOptimizer optimizer;

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolver>(g2o::make_unique<LinearSolver>()));
  optimizer.setAlgorithm(solver);

  VertexParams* params = new VertexParams();
  params->setId(0);
  params->setEstimate(Eigen::Vector2d(1, 1));
  optimizer.addVertex(params);

  for (int i = 0; i < num_points; ++i)
  {
    EdgePointOnCurve* e = new EdgePointOnCurve;
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    e->setVertex(0, params);
    e->setMeasurement(points[i]);
    optimizer.addEdge(e);
  }

  // perform the optimization
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(20);

  // print out the result
  std::cout << "Target curve" << std::endl;
  std::cout << "25 - x ^ 2" << std::endl;
  std::cout << "Iterative least squares solution" << std::endl;
  std::cout << "a      = " << params->estimate()(0) << std::endl;
  std::cout << "b      = " << params->estimate()(1) << std::endl;

  // clean up
  delete[] points;

  return 0;
}
