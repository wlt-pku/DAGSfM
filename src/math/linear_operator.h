#ifndef GRAPHSFM_MATH_MATRIX_LINEAR_OPERATOR_H_
#define GRAPHSFM_MATH_MATRIX_LINEAR_OPERATOR_H_

#include <glog/logging.h>

#include <Eigen/Core>

namespace DAGSfM {

// This is an abstract base class for linear operators. It supports
// access to size information and left and right multiply operators. This class
// is inspired by the LinearOperator class in the Ceres Solver library:
// www.ceres-solver.org
class LinearOperator {
 public:
  virtual ~LinearOperator() = default;

  // y = y + Ax;
  virtual void RightMultiply(const Eigen::VectorXd& x,
                             Eigen::VectorXd* y) const = 0;
  // y = y + A'x;
  virtual void LeftMultiply(const Eigen::VectorXd& x,
                            Eigen::VectorXd* y) const = 0;

  virtual int num_rows() const = 0;
  virtual int num_cols() const = 0;
};

}  // namespace DAGSfM

#endif  // GRAPHSFM_MATH_MATRIX_LINEAR_OPERATOR_H_
