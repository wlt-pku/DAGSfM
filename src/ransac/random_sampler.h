#ifndef RANSAC_RANDOM_SAMPLER_H
#define RANSAC_RANDOM_SAMPLER_H

#include <stdlib.h>

#include <algorithm>
#include <memory>
#include <numeric>
#include <vector>

#include "ransac/sampler.h"
#include "util/random.h"

namespace DAGSfM {

// Random sampler used for RANSAC. This is guaranteed to generate a unique
// sample by performing a Fisher-Yates sampling.
class RandomSampler : public Sampler {
 public:
  RandomSampler(const std::shared_ptr<RandomNumberGenerator>& rng,
                const int min_num_samples);
  ~RandomSampler() {}

  bool Initialize(const int num_datapoints) override;

  // Samples the input variable data and fills the vector subset with the
  // random samples.
  bool Sample(std::vector<int>* subset_indices) override;

 private:
  std::vector<int> sample_indices_;
};

}  // namespace DAGSfM

#endif  // RANSAC_RANDOM_SAMPLER_H
