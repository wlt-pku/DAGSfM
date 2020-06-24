#include "sfm/twoview_info.h"

#include <ceres/rotation.h>
#include <glog/logging.h>

#include <Eigen/Core>
#include <algorithm>

namespace DAGSfM {

void SwapCameras(TwoViewInfo* twoview_info) {
  // Invert the translation.
  Eigen::Vector3d neg_of_new_position;
  ceres::AngleAxisRotatePoint(twoview_info->rotation_2.data(),
                              twoview_info->position_2.data(),
                              neg_of_new_position.data());
  twoview_info->position_2 = -neg_of_new_position;

  // Invert the rotation.
  twoview_info->rotation_2 *= -1.0;
}

}  // namespace DAGSfM
