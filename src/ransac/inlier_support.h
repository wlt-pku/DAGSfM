#ifndef RANSAC_INLIER_SUPPORT_H
#define RANSAC_INLIER_SUPPORT_H

#include <algorithm>
#include <vector>

#include "ransac/quality_measurement.h"

namespace DAGSfM {
// Assess quality of data by whether each error residual is less than an error
// threshold. If it is below the threshold, it is considered an inlier.
class InlierSupport : public QualityMeasurement {
 public:
  explicit InlierSupport(const double error_thresh)
      : QualityMeasurement(error_thresh) {}
  ~InlierSupport() {}

  // Count the number of inliers in the data and return the cost such that lower
  // cost is better.
  double ComputeCost(const std::vector<double>& residuals,
                     std::vector<int>* inliers) override {
    inliers->reserve(residuals.size());
    int size = static_cast<int>(residuals.size());
    for (int i = 0; i < size; i++) {
      if (residuals[i] < this->error_thresh_) {
        inliers->emplace_back(i);
      }
    }
    return residuals.size() - inliers->size();
  }
};

}  // namespace DAGSfM

#endif  // RANSAC_INLIER_SUPPORT_H
