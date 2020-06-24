#ifndef SRC_CLUSTERING_HYBRID_CLUSTER_H_
#define SRC_CLUSTERING_HYBRID_CLUSTER_H_

#include "clustering/cluster.h"

namespace DAGSfM {

class HybridCluster : public Cluster {
 public:
  virtual std::unordered_map<int, int> ComputeCluster(
      const std::vector<std::pair<int, int>>& edges,
      const std::vector<int>& weights, const int num_partitions) override;
};

}  // namespace DAGSfM

#endif