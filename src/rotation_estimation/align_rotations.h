#ifndef SRC_ROTATION_ESTIMATION_ALIGN_ROTATIONS_H_
#define SRC_ROTATION_ESTIMATION_ALIGN_ROTATIONS_H_

#include <Eigen/Core>
#include <vector>

namespace DAGSfM {

// Rotates the "rotation" set of orientations such that the orientations are
// most closely aligned in an L2 sense. That is, "rotation" is transformed such
// that R_rotation * R_gt_rotation^t is minimized.
void AlignRotations(const std::vector<Eigen::Vector3d>& gt_rotation,
                    std::vector<Eigen::Vector3d>* rotation);

}  // namespace DAGSfM

#endif  // GRAPHSFM_SFM_TRANSFORMATION_ALIGN_ROTATIONS_H_
