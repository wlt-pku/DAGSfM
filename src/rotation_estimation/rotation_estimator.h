#ifndef SRC_ROTATION_ESTIMATION_ROTATION_ESTIMATOR_H_
#define SRC_ROTATION_ESTIMATION_ROTATION_ESTIMATOR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>

#include "math/util.h"
#include "rotation_estimation/align_rotations.h"
#include "sfm/twoview_info.h"
#include "util/map_util.h"
#include "util/random.h"
#include "util/types.h"
#include "util/util.h"

using namespace colmap;

namespace DAGSfM {

namespace geometry {
static RandomNumberGenerator rng(56);

// Computes R_ij = R_j * R_i^t.
inline Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2,
    const double noise) {
  const Eigen::Matrix3d noisy_rotation =
      Eigen::AngleAxisd(DegToRad(noise), rng.RandVector3d().normalized())
          .toRotationMatrix();

  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(noisy_rotation * rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

// Computes R_ij = R_j * R_i^t.
inline Eigen::Vector3d RelativeRotationFromTwoRotations(
    const Eigen::Vector3d& rotation1, const Eigen::Vector3d& rotation2) {
  Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
  ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
  ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

  const Eigen::AngleAxisd relative_rotation(rotation_matrix2 *
                                            rotation_matrix1.transpose());
  return relative_rotation.angle() * relative_rotation.axis();
}

// Aligns rotations to the ground truth rotations via a similarity
// transformation.
inline void AlignOrientations(
    const std::unordered_map<image_t, Eigen::Vector3d>& gt_rotations,
    std::unordered_map<image_t, Eigen::Vector3d>* rotations) {
  // Collect all rotations into a vector.
  std::vector<Eigen::Vector3d> gt_rot, rot;
  std::unordered_map<int, int> index_to_view_id;
  int current_index = 0;
  for (const auto& gt_rotation : gt_rotations) {
    gt_rot.emplace_back(gt_rotation.second);
    rot.emplace_back(FindOrDie(*rotations, gt_rotation.first));

    index_to_view_id[current_index] = gt_rotation.first;
    ++current_index;
  }

  AlignRotations(gt_rot, &rot);

  for (unsigned int i = 0; i < rot.size(); i++) {
    const image_t view_id = FindOrDie(index_to_view_id, i);
    (*rotations)[view_id] = rot[i];
  }
}

// return R_j = R_ij * R_i.
inline Eigen::Vector3d ApplyRelativeRotation(
    const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& relative_rotation) {
  Eigen::Vector3d rotation2;
  Eigen::Matrix3d rotation1_matrix, relative_rotation_matrix;
  ceres::AngleAxisToRotationMatrix(
      rotation1.data(), ceres::ColumnMajorAdapter3x3(rotation1_matrix.data()));
  ceres::AngleAxisToRotationMatrix(
      relative_rotation.data(),
      ceres::ColumnMajorAdapter3x3(relative_rotation_matrix.data()));

  const Eigen::Matrix3d rotation2_matrix =
      relative_rotation_matrix * rotation1_matrix;
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(rotation2_matrix.data()), rotation2.data());
  return rotation2;
}

}  // namespace geometry

// The recommended type of rotations solver is the Robust L1-L2 method. This
// method is scalable, extremely accurate, and very efficient. See the
// global_pose_estimation directory for more details.
enum class GlobalRotationEstimatorType {
  ROBUST_L1L2 = 0,
  NONLINEAR = 1,
  LAGRANGIAN_DUAL = 2
};

// A generic class defining the interface for global rotation estimation
// methods. These methods take in as input the relative pairwise orientations
// and output estimates for the global orientation of each view.
class RotationEstimator {
 public:
  RotationEstimator() {}
  virtual ~RotationEstimator() {}
  // Input the view pairs containing relative rotations between matched
  // geometrically verified views and outputs a rotation estimate for each view.
  //
  // Returns true if the rotation estimation was a success, false if there was a
  // failure. If false is returned, the contents of rotations are undefined.
  virtual bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* rotations) = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(RotationEstimator);
};

}  // namespace DAGSfM

#endif  // SRC_ROTATION_ESTIMATION_ROTATION_ESTIMATOR_H_
