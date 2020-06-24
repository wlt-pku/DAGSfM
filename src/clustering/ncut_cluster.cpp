#include "clustering/ncut_cluster.h"

#include "base/graph_cut.h"

namespace DAGSfM {

std::unordered_map<int, int> NCutCluster::ComputeCluster(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights, const int num_partitions) {
  if (num_partitions == 1) {
    for (auto node : nodes_) {
      labels_[node] = 0;
    }
  } else {
    cluster_num_ = num_partitions;
    labels_ =
        colmap::ComputeNormalizedMinGraphCut(edges, weights, num_partitions);
  }

  return labels_;
}

}  // namespace DAGSfM