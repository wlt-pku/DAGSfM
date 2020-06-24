#ifndef SRC_GRAPH_SIMILARITY_GRAPH_H_
#define SRC_GRAPH_SIMILARITY_GRAPH_H_

#include <glog/logging.h>

#include <cstdlib>
#include <utility>
#include <vector>

#include "feature/feature.h"
#include "feature/matching.h"
#include "graph/image_graph.h"
#include "util/threading.h"
#include "util/timer.h"

using namespace colmap;

namespace DAGSfM {

struct VocabSimilaritySearchOptions {
  // Number of images to retrieve for each query image.
  int num_images = 100;

  // Number of nearest neighbors to retrieve per query feature.
  int num_nearest_neighbors = 5;

  // Number of nearest-neighbor checks to use in retrieval.
  int num_checks = 256;

  // How many images to return after spatial verification. Set to 0 to turn off
  // spatial verification.
  int num_images_after_verification = 0;

  // The maximum number of features to use for indexing an image. If an
  // image has more features, only the largest-scale features will be indexed.
  int max_num_features = -1;

  int num_threads = 8;

  // Path to the vocabulary tree.
  std::string vocab_tree_path = "";

  void Check();
};

class VocabSimilarityGraph : public Thread, public ImageGraph {
 public:
  VocabSimilarityGraph(const VocabSimilaritySearchOptions& options,
                       const Database& database);

  void Run() override;

 private:
  VocabSimilaritySearchOptions options_;

  FeatureMatcherCache cache_;
};

struct MirrorSimilaritySearchOptions {
  // Number of images to retrieve for each query image.
  int num_images = 100;

  // Script path to execute similarity search.
  std::string script_path = "";

  // dataset path.
  std::string dataset_path = "";

  // Output directory of matching pairs.
  std::string output_dir = "";

  // Path to the mirror library.
  std::string mirror_path = "";

  void Check();
};

class MirrorSimilarityGraph : public Thread, public ImageGraph {
 public:
  MirrorSimilarityGraph(const MirrorSimilaritySearchOptions& options);

  void Run() override;

 private:
  MirrorSimilaritySearchOptions options_;

  void LoadSearchResults();
};

}  // namespace DAGSfM

#endif