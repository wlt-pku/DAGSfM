#ifndef SRC_ROTATION_ESTIMATION_PAIRWISE_ROTATION_ERROR_H_
#define SRC_ROTATION_ESTIMATION_PAIRWISE_ROTATION_ERROR_H_

#include <ceres/rotation.h>

#include <Eigen/Core>

namespace ceres {
class CostFunction;
}  // namespace ceres

namespace DAGSfM {

// The error in two global rotations based on the current estimates for the
// global rotations and the relative rotation such that R{i, j} = R_j * R_i'.
struct PairwiseRotationError {
  PairwiseRotationError(const Eigen::Vector3d& relative_rotation,
                        const double weight);

  // The error is given by the rotation loop error as specified above. We return
  // 3 residuals to give more opportunity for optimization.
  template <typename T>
  bool operator()(const T* rotation1, const T* rotation2, T* residuals) const;

  static ceres::CostFunction* Create(const Eigen::Vector3d& relative_rotation,
                                     const double weight);

  const Eigen::Vector3d relative_rotation_;
  const double weight_;
};

template <typename T>
bool PairwiseRotationError::operator()(const T* rotation1, const T* rotation2,
                                       T* residuals) const {
  // Convert angle axis rotations to rotation matrices.
  Eigen::Matrix<T, 3, 3> rotation1_mat, rotation2_mat;
  Eigen::Matrix3d relative_rotation_mat;
  ceres::AngleAxisToRotationMatrix(
      rotation1, ceres::ColumnMajorAdapter3x3(rotation1_mat.data()));
  ceres::AngleAxisToRotationMatrix(
      rotation2, ceres::ColumnMajorAdapter3x3(rotation2_mat.data()));
  ceres::AngleAxisToRotationMatrix(
      relative_rotation_.data(),
      ceres::ColumnMajorAdapter3x3(relative_rotation_mat.data()));

  // Compute the loop rotation from the two global rotations.
  const Eigen::Matrix<T, 3, 3> loop_rotation_mat =
      rotation2_mat * rotation1_mat.transpose();
  // Compute the error matrix between the expected relative rotation and the
  // observed relative rotation
  const Eigen::Matrix<T, 3, 3> error_rotation_mat =
      loop_rotation_mat * relative_rotation_mat.cast<T>().transpose();
  Eigen::Matrix<T, 3, 1> error_rotation;
  ceres::RotationMatrixToAngleAxis(
      ceres::ColumnMajorAdapter3x3(error_rotation_mat.data()),
      error_rotation.data());
  residuals[0] = weight_ * error_rotation(0);
  residuals[1] = weight_ * error_rotation(1);
  residuals[2] = weight_ * error_rotation(2);

  return true;
}

}  // namespace DAGSfM

#endif  // SRC_ROTATION_ESTIMATION_PAIRWISE_ROTATION_ERROR_H_
