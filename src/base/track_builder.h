#ifndef COLMAP_SRC_BASE_TRACK_BUILDER_H
#define COLMAP_SRC_BASE_TRACK_BUILDER_H

#include <unordered_map>
#include <vector>

#include "base/track.h"
#include "graph/union_find.h"
#include "util/types.h"

using namespace colmap;

namespace DAGSfM {

// Build tracks from feature correspondences across multiple images. Tracks are
// created with the connected components algorithm and have a maximum allowable
// size. If there are multiple features from one image in a track, we do not do
// any intelligent selection and just arbitrarily choose a feature to drop so
// that the tracks are consistent.
class TrackBuilder {
 public:
  // Build tracks for a given series of track elements.
  void Build(const std::vector<TrackElement>& track_elements,
             const std::vector<std::pair<track_t, track_t>>& pair_ids);

  // Remove bad tracks that are too short or have ids collision.
  bool Filter(const int min_track_length = 2, const int max_track_length = 100);

  // Return the number of connected set in the Union Find structure.
  size_t NumTracks() const;

  // Extract consistent tracks.
  std::unordered_map<track_t, std::vector<TrackElement>> GetConsistentTracks()
      const;

 private:
  std::unordered_map<track_t, std::vector<TrackElement>> consistent_tracks_;
};

}  // namespace DAGSfM

#endif