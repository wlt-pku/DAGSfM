#include "clustering/community_detection_cluster.h"

#include <gtest/gtest.h>

#include <iostream>
#include <unordered_set>

#include "util/map_util.h"
#include "util/random.h"
#include "util/types.h"

using namespace DAGSfM;
using namespace colmap;

class ImageClusteringTest : public ::testing::Test {
 protected:
  std::vector<image_t> image_ids_;
  std::vector<std::pair<int, int>> view_pairs_;
  std::vector<int> weights_;

 public:
  void TestImageClustering(const int num_views, const int num_view_pairs) {
    ASSERT_LE(num_views + 1, num_view_pairs);

    // Initialize images
    for (int i = 0; i < num_views; i++) {
      image_ids_.push_back(i);
    }

    // Initialize view pairs
    LOG(INFO) << "Initializing view pairs...";
    CreateViewPairsFromSpanningTree(num_view_pairs);

    LOG(INFO) << "Begin clustering...";
    CommunityDetectionCluster cluster;
    cluster.InitIGraph(view_pairs_, weights_);
    auto labels = cluster.ComputeCluster(view_pairs_, weights_, 0);
    cluster.OutputIGraph("/home/chenyu/Projects/distributed_colmap/build");
    // for (int i = 0; i < num_views; i++) {
    //     std::cout << labels[i] << " ";
    // }
  }

 protected:
  void SetUp() {}

  void CreateViewPairsFromSpanningTree(const int num_view_pairs) {
    std::unordered_set<ImagePair> view_pairs;
    RandomNumberGenerator rng;
    for (size_t i = 1; i < image_ids_.size(); i++) {
      const ImagePair view_id_pair(i - 1, i);
      view_pairs.insert(view_id_pair);
      view_pairs_.push_back(view_id_pair);
      weights_.push_back(rng.RandInt(30, 100));
    }

    // Add random edges
    while (view_pairs_.size() < (uint)num_view_pairs) {
      const ImagePair view_id_pair(rng.RandInt(0, image_ids_.size() - 1),
                                   rng.RandInt(0, image_ids_.size() - 1));

      // Ensure the first id is smaller than second id &&
      // do not add the views that already exists
      if (view_id_pair.first >= view_id_pair.second ||
          ContainsKey(view_pairs, view_id_pair)) {
        continue;
      }

      view_pairs.insert(view_id_pair);
      view_pairs_.push_back(view_id_pair);
      weights_.push_back(rng.RandInt(30, 100));
    }
  }
};

TEST_F(ImageClusteringTest, TestThirtyImages) { TestImageClustering(30, 40); }

TEST_F(ImageClusteringTest, TestOneHundredImages) {
  TestImageClustering(100, 2000);
}

TEST_F(ImageClusteringTest, TestOneThousandImages) {
  TestImageClustering(1000, 100000);
}

TEST_F(ImageClusteringTest, TestFiveThousandImages) {
  TestImageClustering(5000, 1000000);
}

TEST_F(ImageClusteringTest, TestTenThousandImages) {
  TestImageClustering(10000, 1000000);
}

TEST_F(ImageClusteringTest, TestOneHundredThousandImages) {
  TestImageClustering(100000, 10000000);
}
