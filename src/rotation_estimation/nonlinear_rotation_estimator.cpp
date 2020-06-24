#include "rotation_estimation/nonlinear_rotation_estimator.h"

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <memory>
#include <unordered_map>

#include "rotation_estimation/pairwise_rotation_error.h"
#include "util/hash.h"
#include "util/map_util.h"
#include "util/types.h"

namespace DAGSfM {

bool NonlinearRotationEstimator::EstimateRotations(
    const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
    std::unordered_map<image_t, Eigen::Vector3d>* global_orientations) {
  CHECK_NOTNULL(global_orientations);
  if (global_orientations->size() == 0) {
    LOG(INFO) << "Skipping nonlinear rotation optimization because no "
                 "initialization was provivded.";
    return false;
  }
  if (view_pairs.size() == 0) {
    LOG(INFO) << "Skipping nonlinear rotation optimization because no "
                 "relative rotation constraints were provivded.";
    return false;
  }

  // Set up the problem and loss function.
  std::unique_ptr<ceres::Problem> problem(new ceres::Problem());
  ceres::LossFunction* loss_function =
      new ceres::SoftLOneLoss(robust_loss_width_);

  for (const auto& view_pair : view_pairs) {
    const ImagePair& view_id_pair = view_pair.first;
    Eigen::Vector3d* rotation1 =
        FindOrNull(*global_orientations, view_id_pair.first);
    Eigen::Vector3d* rotation2 =
        FindOrNull(*global_orientations, view_id_pair.second);

    // Do not add the relative rotation constaint if it requires an orientation
    // that we do not have an initialization for.
    if (rotation1 == nullptr || rotation2 == nullptr) {
      continue;
    }

    ceres::CostFunction* cost_function =
        PairwiseRotationError::Create(view_pair.second.rotation_2, 1.0);
    problem->AddResidualBlock(cost_function, loss_function, rotation1->data(),
                              rotation2->data());
  }

  // The problem should be relatively sparse so sparse cholesky is a good
  // choice.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 200;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  VLOG(1) << summary.FullReport();
  return true;
}

}  // namespace DAGSfM
