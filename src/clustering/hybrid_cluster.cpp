#include "clustering/hybrid_cluster.h"

#include <unordered_set>

#include "base/graph_cut.h"
#include "clustering/community_detection_cluster.h"
#include "clustering/ncut_cluster.h"

namespace DAGSfM {

std::unordered_map<int, int> HybridCluster::ComputeCluster(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<int>& weights, const int num_partitions) {
  std::unordered_map<int, int> labels;

  igraph_vector_t modularity;
  igraph_vector_t membership;
  igraph_matrix_t merges;

  igraph_vector_init(&modularity, 0);
  igraph_vector_init(&membership, 0);
  igraph_matrix_init(&merges, 0, 0);

  // Community detection.
  igraph_community_fastgreedy(&igraph_, &i_weights_, &merges, &modularity,
                              &membership);

  for (uint i = 0; i < igraph_vector_size(&membership); i++) {
    labels[nodes_[i]] = (int)VECTOR(membership)[i];
  }

  igraph_vector_destroy(&modularity);
  igraph_vector_destroy(&membership);
  igraph_matrix_destroy(&merges);

  int max_cluster_id = -1;
  for (auto label : labels) {
    max_cluster_id = std::max(max_cluster_id, label.second);
  }

  std::vector<std::vector<std::pair<int, int>>> edges_groups;
  std::vector<std::vector<int>> weights_groups;
  std::vector<std::unordered_set<int>> nodes_groups;
  edges_groups.resize(max_cluster_id + 1);
  weights_groups.resize(max_cluster_id + 1);
  nodes_groups.resize(max_cluster_id + 1);

  // Grouping the edges for each cluster after community detection.
  for (size_t k = 0; k < edges.size(); k++) {
    const int i = edges[k].first;
    const int j = edges[k].second;
    const int cluster_id1 = labels.at(i);
    const int cluster_id2 = labels.at(j);

    if (cluster_id1 == cluster_id2) {
      edges_groups[cluster_id1].push_back(std::make_pair(i, j));
      weights_groups[cluster_id1].push_back(weights[k]);
      nodes_groups[cluster_id1].insert(i);
      nodes_groups[cluster_id1].insert(j);
    }
  }

  // Cluster the edges after community detection.
  std::unordered_map<int, int> labels_;
  const int num_images_ub = labels.size() / num_partitions;
  int max_label = 0;
  for (uint k = 0; k < edges_groups.size(); k++) {
    const int local_cluster_num = nodes_groups[k].size() / num_images_ub;
    if (local_cluster_num == 0) continue;

    cluster_num_ += local_cluster_num;

    // NCutCluster ncut_cluster;
    // ncut_cluster.InitIGraph(edges_groups[k], weights_groups[k]);
    const std::unordered_map<int, int> local_labels =
        colmap::ComputeNormalizedMinGraphCut(edges_groups[k], weights_groups[k],
                                             local_cluster_num);
    // ncut_cluster.ComputeCluster(edges_groups[k], weights_groups[k],
    // local_cluster_num);

    // global label = local label + max_label
    for (auto label : local_labels) {
      labels_[label.first] = label.second + max_label;
    }

    int max_local_label = 0;
    for (auto label : local_labels) {
      max_local_label = std::max(max_local_label, label.second + 1);
    }
    max_label += max_local_label;
  }

  return labels_;
}

}  // namespace DAGSfM