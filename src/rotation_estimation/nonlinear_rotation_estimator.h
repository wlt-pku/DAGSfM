#ifndef SRC_ROTATION_ESTIMATION_NONLINEAR_ROTATION_ESTIMATOR_H_
#define SRC_ROTATION_ESTIMATION_NONLINEAR_ROTATION_ESTIMATOR_H_

#include <Eigen/Core>
#include <unordered_map>

#include "rotation_estimation/rotation_estimator.h"
#include "util/hash.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

// Computes the global rotations given relative rotations and an initial guess
// for the global orientations. Nonlinear optimization is performed with Ceres
// using a SoftL1 loss function to be robust to outliers.
class NonlinearRotationEstimator : public RotationEstimator {
 public:
  NonlinearRotationEstimator() : robust_loss_width_(0.1) {}
  explicit NonlinearRotationEstimator(const double robust_loss_width)
      : robust_loss_width_(robust_loss_width) {}

  // Estimates the global orientations of all views based on an initial
  // guess. Returns true on successful estimation and false otherwise.
  bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* global_orientations);

 private:
  const double robust_loss_width_;
};

}  // namespace DAGSfM

#endif  // SRC_ROTATION_ESTIMATION_NONLINEAR_ROTATION_ESTIMATOR_H_