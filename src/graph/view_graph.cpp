#include "graph/view_graph.h"

namespace DAGSfM {

bool ViewGraph::AddTwoViewGeometry(const ImagePair& image_pair,
                                   const TwoViewInfo& twoview_info) {
  if (twoview_geometries_.count(image_pair) > 0) {
    return false;
  }

  twoview_geometries_.emplace(image_pair, twoview_info);

  return true;
}

bool ViewGraph::RemoveTwoViewGeometry(const ImagePair& image_pair) {
  if (twoview_geometries_.count(image_pair) == 0) {
    return false;
  }

  twoview_geometries_.erase(image_pair);
  return true;
}

void ViewGraph::TwoviewGeometriesToImagePairs() {
  image_pairs_.clear();
  image_pairs_.reserve(twoview_geometries_.size());
  for (const auto& view_pair : twoview_geometries_) {
    image_pairs_.emplace_back(view_pair.first.first, view_pair.first.second);
  }
}

}  // namespace DAGSfM