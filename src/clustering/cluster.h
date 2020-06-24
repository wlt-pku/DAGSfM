#ifndef SRC_CLUSTERING_CLUSTER_H_
#define SRC_CLUSTERING_CLUSTER_H_

#include <glog/logging.h>
#include <igraph/igraph.h>

#include <cstdio>
#include <unordered_map>

#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

enum ClusterType { NCUT, KMEANS, SPECTRAL, COMMUNITY_DETECTION, HYBRID };

class Cluster {
 protected:
  std::unordered_map<int, int> labels_;

  igraph_t igraph_;

  igraph_vector_t i_edges_;

  igraph_vector_t i_weights_;

  std::vector<int> nodes_;

  int cluster_num_;

 public:
  Cluster();
  ~Cluster();

  virtual std::unordered_map<int, int> ComputeCluster(
      const std::vector<std::pair<int, int>>& edges,
      const std::vector<int>& weights, const int num_partitions) = 0;

  int ClusterNum() const;

  bool InitIGraph(const std::vector<std::pair<int, int>>& edges,
                  const std::vector<int>& weights);

  bool OutputIGraph(const std::string graph_dir,
                    const std::string image_path = "") const;
};

}  // namespace DAGSfM

#endif