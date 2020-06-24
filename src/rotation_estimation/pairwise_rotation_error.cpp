
#include "rotation_estimation/pairwise_rotation_error.h"

#include <ceres/ceres.h>

#include <Eigen/Core>

namespace DAGSfM {

PairwiseRotationError::PairwiseRotationError(
    const Eigen::Vector3d& relative_rotation, const double weight)
    : relative_rotation_(relative_rotation), weight_(weight) {}

ceres::CostFunction* PairwiseRotationError::Create(
    const Eigen::Vector3d& relative_rotation, const double weight) {
  return new ceres::AutoDiffCostFunction<PairwiseRotationError, 3, 3, 3>(
      new PairwiseRotationError(relative_rotation, weight));
}

}  // namespace DAGSfM