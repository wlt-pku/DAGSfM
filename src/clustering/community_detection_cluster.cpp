#include "clustering/community_detection_cluster.h"

#include <igraph/igraph.h>

#include <algorithm>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace DAGSfM {
std::unordered_map<int, int> CommunityDetectionCluster::ComputeCluster(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights, const int num_partitions) {
  igraph_vector_t modularity;
  igraph_vector_t membership;
  igraph_matrix_t merges;

  igraph_vector_init(&modularity, 0);
  igraph_vector_init(&membership, 0);
  igraph_matrix_init(&merges, 0, 0);

  // Community detection.
  igraph_community_fastgreedy(&igraph_, &i_weights_, &merges, &modularity,
                              &membership);

  // std::unordered_map<int, int> labels;
  for (uint i = 0; i < igraph_vector_size(&membership); i++) {
    labels_[nodes_[i]] = (int)VECTOR(membership)[i];
    cluster_num_ = std::max(cluster_num_, labels_[nodes_[i]] + 1);
  }

  igraph_vector_destroy(&modularity);
  igraph_vector_destroy(&membership);
  igraph_matrix_destroy(&merges);

  return labels_;
}

}  // namespace DAGSfM