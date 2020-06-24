#include "base/track_builder.h"

#include <glog/logging.h>

namespace DAGSfM {

void TrackBuilder::Build(
    const std::vector<TrackElement>& track_elements,
    const std::vector<std::pair<track_t, track_t>>& pair_ids) {
  LOG(INFO) << "Build tracks";
  graph::UnionFind uf(track_elements.size());
  for (size_t i = 0; i < track_elements.size(); i++) {
    uf.Union(pair_ids[i].first, pair_ids[i].second);
  }

  for (size_t i = 0; i < track_elements.size(); i++) {
    const size_t parent_id = uf.FindRoot(i);
    consistent_tracks_[parent_id].emplace_back(track_elements[i]);
  }
}

bool TrackBuilder::Filter(const int min_track_length,
                          const int max_track_length) {
  LOG(INFO) << "Filter tracks.";
  size_t num_small_tracks = 0;
  size_t num_inconsistent_track_elements = 0;
  LOG(INFO) << "tracks size: " << consistent_tracks_.size();
  for (auto track_it = consistent_tracks_.begin();
       track_it != consistent_tracks_.end();) {
    LOG(INFO) << track_it->first;
    // If track.length < min_track_length or track.length > max_track_length,
    // we should discard this track.
    if (track_it->second.size() < min_track_length ||
        track_it->second.size() > max_track_length) {
      LOG(INFO) << "Remove short tracks";
      consistent_tracks_.erase(track_it++);
      num_small_tracks++;
      continue;
    }

    const std::vector<TrackElement>& candidate_track = track_it->second;
    std::vector<TrackElement> consistent_track;
    consistent_track.reserve(track_it->second.size());

    LOG(INFO) << "Remove inconsistent tracks";
    std::unordered_set<image_t> image_ids;
    for (size_t i = 0; i < candidate_track.size(); i++) {
      const TrackElement& track_element = candidate_track[i];
      // Do not add the track_element if the track already contains a
      // track element from the same image.
      if (image_ids.count(track_element.image_id) != 0) {
        num_inconsistent_track_elements++;
        continue;
      }

      image_ids.insert(track_element.image_id);
      consistent_track.emplace_back(track_element);
    }

    if (candidate_track.size() != consistent_track.size()) {
      LOG(INFO) << "Re-assign tracks elements";
      consistent_tracks_[track_it->first].clear();
      consistent_tracks_[track_it->first].assign(consistent_track.begin(),
                                                 consistent_track.end());
    }

    ++track_it;
  }

  LOG(INFO) << num_small_tracks << " small tracks are removed";
  LOG(INFO) << num_inconsistent_track_elements
            << " inconsistent track element are removed.";
}

size_t TrackBuilder::NumTracks() const { return consistent_tracks_.size(); }

std::unordered_map<track_t, std::vector<TrackElement>>
TrackBuilder::GetConsistentTracks() const {
  return consistent_tracks_;
}

}  // namespace DAGSfM