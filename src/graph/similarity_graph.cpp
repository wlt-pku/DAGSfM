#include "graph/similarity_graph.h"

#include <fstream>
#include <unordered_set>

#include "feature/utils.h"
#include "omp.h"
#include "retrieval/utils.h"
#include "retrieval/visual_index.h"
#include "util/hash.h"
#include "util/misc.h"
#include "util/threading.h"
#include "util/timer.h"

namespace DAGSfM {

namespace {

void PrintElapsedTime(const Timer& timer) {
  std::cout << StringPrintf(" in %.3fs", timer.ElapsedSeconds()) << std::endl;
}

void IndexImagesInVisualIndex(const int num_threads, const int num_checks,
                              const int max_num_features,
                              const std::vector<image_t>& image_ids,
                              Thread* thread, FeatureMatcherCache* cache,
                              retrieval::VisualIndex<>* visual_index) {
  retrieval::VisualIndex<>::IndexOptions index_options;
  index_options.num_threads = num_threads;
  index_options.num_checks = num_checks;

  for (size_t i = 0; i < image_ids.size(); ++i) {
    if (thread->IsStopped()) {
      return;
    }

    Timer timer;
    timer.Start();

    std::cout << StringPrintf("Indexing image [%d/%d]", i + 1, image_ids.size())
              << std::flush;

    auto keypoints = cache->GetKeypoints(image_ids[i]);
    auto descriptors = cache->GetDescriptors(image_ids[i]);
    if (max_num_features > 0 && descriptors.rows() > max_num_features) {
      ExtractTopScaleFeatures(&keypoints, &descriptors, max_num_features);
    }

    visual_index->Add(index_options, image_ids[i], keypoints, descriptors);

    PrintElapsedTime(timer);
  }

  // Compute the TF-IDF weights, etc.
  visual_index->Prepare();
}

}  // namespace

void VocabSimilaritySearchOptions::Check() {
  CHECK_GT(num_images, 0);
  CHECK_GT(vocab_tree_path.size(), 0);
}

VocabSimilarityGraph::VocabSimilarityGraph(
    const VocabSimilaritySearchOptions& options, const Database& database)
    : options_(options), cache_(5 * options_.num_images, &database) {}

void VocabSimilarityGraph::Run() {
  cache_.Setup();

  // Read the pre-trained vocabulary tree from disk.
  retrieval::VisualIndex<> visual_index;
  visual_index.Read(options_.vocab_tree_path);

  const std::vector<image_t> all_image_ids = cache_.GetImageIds();

  // Index all images in the visual index.
  IndexImagesInVisualIndex(options_.num_threads, options_.num_checks,
                           options_.max_num_features, all_image_ids, this,
                           &cache_, &visual_index);

  struct Retrieval {
    image_t image_id = kInvalidImageId;
    std::vector<retrieval::ImageScore> image_scores;
  };

  // Create a thread pool to retrieve the nearest neighbors.
  ThreadPool retrieval_thread_pool(options_.num_threads);
  JobQueue<Retrieval> retrieval_queue(options_.num_threads);

  // The retrieval thread kernel function. Note that the descriptors should be
  // extracted outside of this function sequentially to avoid any concurrent
  // access to the database causing race conditions.
  retrieval::VisualIndex<>::QueryOptions query_options;
  query_options.max_num_images = options_.num_images;
  query_options.num_neighbors = options_.num_nearest_neighbors;
  query_options.num_checks = options_.num_checks;
  query_options.num_images_after_verification =
      options_.num_images_after_verification;

  auto QueryFunc = [&](const image_t image_id) {
    auto keypoints = cache_.GetKeypoints(image_id);
    auto descriptors = cache_.GetDescriptors(image_id);
    if (options_.max_num_features > 0 &&
        descriptors.rows() > options_.max_num_features) {
      ExtractTopScaleFeatures(&keypoints, &descriptors,
                              options_.max_num_features);
    }

    Retrieval retrieval;
    retrieval.image_id = image_id;

    Timer timer;
    timer.Start();
    std::cout << "; Quering image " << image_id;
    visual_index.Query(query_options, keypoints, descriptors,
                       &retrieval.image_scores);

    PrintElapsedTime(timer);
    CHECK(retrieval_queue.Push(retrieval));
  };

  // Initially, make all retrieval threads busy and continue with the matching.
  size_t image_idx = 0;
  const size_t init_num_tasks =
      std::min(all_image_ids.size(), 2 * retrieval_thread_pool.NumThreads());
  for (; image_idx < init_num_tasks; ++image_idx) {
    retrieval_thread_pool.AddTask(QueryFunc, all_image_ids[image_idx]);
  }

  std::vector<std::pair<image_t, image_t>> image_pairs;

  // Pop the finished retrieval results and enqueue them for feature matching.
  for (size_t i = 0; i < all_image_ids.size(); ++i) {
    if (this->IsStopped()) {
      retrieval_queue.Stop();
      return;
    }

    std::cout << StringPrintf("Retrieving image [%d/%d]", i + 1,
                              all_image_ids.size())
              << std::flush;

    // Push the next image to the retrieval queue.
    if (image_idx < all_image_ids.size()) {
      retrieval_thread_pool.AddTask(QueryFunc, all_image_ids[image_idx]);
      image_idx += 1;
    }

    // Pop the next results from the retrieval queue.
    const auto retrieval = retrieval_queue.Pop();
    CHECK(retrieval.IsValid());

    const auto& image_id = retrieval.Data().image_id;
    const auto& image_scores = retrieval.Data().image_scores;

    // Compose the image pairs from the scores.
    // image_pairs_.reserve(image_scores.size());
    for (const auto image_score : image_scores) {
      if (image_id < image_score.image_id) {
        image_pairs_.emplace_back(image_id, image_score.image_id);
        scores_.push_back(image_score.score * 1e3);
      }
    }
  }
}

void MirrorSimilaritySearchOptions::Check() {
  CHECK_GT(num_images, 0);
  CHECK_GT(script_path.size(), 0);
  CHECK_GT(dataset_path.size(), 0);
  CHECK_GT(output_dir.size(), 0);
  CHECK_GT(mirror_path.size(), 0);
}

MirrorSimilarityGraph::MirrorSimilarityGraph(
    const MirrorSimilaritySearchOptions& options)
    : options_(options) {
  options_ = options;
  options_.Check();
}

void MirrorSimilarityGraph::Run() {
  std::string command = options_.script_path + " " + options_.dataset_path +
                        " " + options_.output_dir + " " +
                        std::to_string(options_.num_images) + " " +
                        options_.mirror_path;

  colmap::Timer timer;
  LOG(INFO) << "Similarity Searching by MIRROR Models.";
  timer.Start();
  { std::system(command.c_str()); }
  LoadSearchResults();
  timer.Pause();
  LOG(INFO) << "Time elapsed: " << timer.ElapsedSeconds();
}

void MirrorSimilarityGraph::LoadSearchResults() {
  std::ifstream in_image_list(
      colmap::JoinPaths(options_.dataset_path, "image_list.txt"));
  if (!in_image_list.is_open()) {
    LOG(ERROR) << "Cannot open image list file in " << options_.dataset_path;
    return;
  }

  std::string abs_image_name;
  image_t image_index = 0;
  while (in_image_list >> abs_image_name) {
    std::string image_name = colmap::GetPathBaseName(abs_image_name);

    image_ids_.emplace_back(image_index);
    image_id_to_name_.emplace(image_index, image_name);
    image_name_to_id_.emplace(image_name, image_index);

    image_index++;
  }
  in_image_list.close();

  std::ifstream in_match_pairs(
      colmap::JoinPaths(options_.output_dir, "match_pairs"));
  if (!in_match_pairs.is_open()) {
    LOG(ERROR) << "Cannot open matching pairs in " << options_.output_dir;
    return;
  }

  size_t src, dst;
  double score;
  std::unordered_set<ImagePair> image_pair_set;
  while (in_match_pairs >> src >> dst >> score) {
    if (src == dst) {
      continue;
    }

    const ImagePair image_pair =
        (src < dst) ? ImagePair(src, dst) : ImagePair(dst, src);
    if (image_pair_set.count(image_pair) > 0) {
      continue;
    }

    image_pair_set.insert(image_pair);
    image_pairs_.emplace_back(image_pair);
    scores_.emplace_back(score);
  }
  in_match_pairs.close();
}

}  // namespace DAGSfM