#ifndef SRC_CLUSTERING_SPECTRAL_CLUSTER_H_
#define SRC_CLUSTERING_SPECTRAL_CLUSTER_H_

#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <vector>

#include "clustering/cluster.h"

namespace DAGSfM {

class SpectralCluster : public Cluster {
 public:
  virtual std::unordered_map<int, int> ComputeCluster(
      const std::vector<std::pair<int, int>>& edges,
      const std::vector<int>& weights, const int num_partitions) override;

 private:
  Eigen::SparseMatrix<double> ComputeLaplacian(
      const Eigen::SparseMatrix<double>& S,
      const std::unordered_map<int, int>& degrees) const;

  std::unordered_map<int, int> node_mapper_;
};

}  // namespace DAGSfM

#endif