#ifndef SRC_ROTATION_ESTIMATION_ROBUST_ROTATION_ESTIMATOR_H_
#define SRC_ROTATION_ESTIMATION_ROBUST_ROTATION_ESTIMATOR_H_

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <unordered_map>

#include "rotation_estimation/rotation_estimator.h"
#include "util/hash.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {
class TwoViewInfo;

// Computes the global rotations given relative rotations and an initial guess
// for the global orientations. The robust algorithm of "Efficient and Large
// Scale Rotation Averaging" by Chatterjee and Govindu (ICCV 2013) is used to
// obtain accurate solutions that are robust to outliers.
//
// The general strategy of this algorithm is to minimize the relative rotation
// error (using the difference between relative rotations and the corresponding
// global rotations) with L1 minimization first, then a reweighted least
// squares. The L1 minimization is relatively slow, but provides excellent
// robustness to outliers. Then the L2 minimization (which is much faster) can
// refine the solution to be very accurate.
class RobustRotationEstimator : public RotationEstimator {
 public:
  struct Options {
    // Maximum number of times to run L1 minimization. L1 is very slow (compared
    // to L2), but is very robust to outliers. Typically only a few iterations
    // are needed in order for the solution to reside within the cone of
    // convergence for L2 solving.
    int max_num_l1_iterations = 5;

    // Average step size threshold to terminate the L1 minimization
    double l1_step_convergence_threshold = 0.001;

    // The number of iterative reweighted least squares iterations to perform.
    int max_num_irls_iterations = 100;

    // Average step size threshold to termininate the IRLS minimization
    double irls_step_convergence_threshold = 0.001;

    // This is the point where the Huber-like cost function switches from L1 to
    // L2.
    double irls_loss_parameter_sigma = DegToRad(5.0);
  };

  explicit RobustRotationEstimator(const Options& options)
      : options_(options) {}

  // Estimates the global orientations of all views based on an initial
  // guess. Returns true on successful estimation and false otherwise.
  bool EstimateRotations(
      const std::unordered_map<ImagePair, TwoViewInfo>& view_pairs,
      std::unordered_map<image_t, Eigen::Vector3d>* global_orientations);

  // An alternative interface is to instead add relative rotation constraints
  // one by one with AddRelativeRotationConstraint, then call the
  // EstimateRotations interface below. This allows the caller to add multiple
  // constraints for the same view id pair, which may lead to more accurate
  // rotation estimates. Please see the following reference for an example of
  // how to obtain multiple constraints for pairs of views:
  //
  //   "Parallel Structure from Motion from Local Increment to Global Averaging"
  //   by Zhu et al (Arxiv 2017). https://arxiv.org/abs/1702.08601
  void AddRelativeRotationConstraint(const ImagePair& view_id_pair,
                                     const Eigen::Vector3d& relative_rotation);

  // Given the relative rotation constraints added with
  // AddRelativeRotationConstraint, this method returns the robust estimation of
  // global camera orientations. Like the method above, this requires an initial
  // estimate of the global orientations.
  bool EstimateRotations(
      std::unordered_map<image_t, Eigen::Vector3d>* global_orientations);

 protected:
  // Sets up the sparse linear system such that dR_ij = dR_j - dR_i. This is the
  // first-order approximation of the angle-axis rotations. This should only be
  // called once.
  void SetupLinearSystem();

  // Performs the L1 robust loss minimization.
  bool SolveL1Regression();

  // Performs the iteratively reweighted least squares.
  bool SolveIRLS();

  // Updates the global rotations based on the current rotation change.
  void UpdateGlobalRotations();

  // Computes the relative rotation (tangent space) residuals based on the
  // current global orientation estimates.
  void ComputeResiduals();

  // Computes the average size of the most recent step of the algorithm.
  // The is the average over all non-fixed global_orientations_ of their
  // rotation magnitudes.
  double ComputeAverageStepSize();

  // We keep one of the rotations as constant to remove the ambiguity of the
  // linear system.
  static const int kConstantRotationIndex = -1;

  // Options for the solver.
  const Options options_;

  // The pairwise relative rotations used to compute the global rotations.
  std::vector<std::pair<ImagePair, Eigen::Vector3d> > relative_rotations_;

  // The global orientation estimates for each camera.
  std::unordered_map<image_t, Eigen::Vector3d>* global_orientations_;

  // Map of image_ts to the corresponding positions of the view's orientation in
  // the linear system.
  std::unordered_map<image_t, int> view_id_to_index_;

  // The sparse matrix used to maintain the linear system. This is matrix A in
  // Ax = b.
  Eigen::SparseMatrix<double> sparse_matrix_;

  // x in the linear system Ax = b.
  Eigen::VectorXd tangent_space_step_;

  // b in the linear system Ax = b.
  Eigen::VectorXd tangent_space_residual_;
};

}  // namespace DAGSfM

#endif  // SRC_ROTATION_ESTIMATION_ROBUST_ROTATION_ESTIMATOR_H_
