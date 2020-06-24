#ifndef SRC_GRAPH_VIEW_GRAPH_H_
#define SRC_GRAPH_VIEW_GRAPH_H_

#include "graph/image_graph.h"
#include "sfm/twoview_info.h"
#include "util/hash.h"

namespace DAGSfM {

class ViewGraph : public ImageGraph {
 public:
  bool AddTwoViewGeometry(const ImagePair& image_pair,
                          const TwoViewInfo& twoview_info);
  bool RemoveTwoViewGeometry(const ImagePair& image_pair);

  void TwoviewGeometriesToImagePairs();

  inline std::unordered_map<ImagePair, TwoViewInfo>& TwoViewGeometries();
  inline const std::unordered_map<ImagePair, TwoViewInfo>& TwoViewGeometries()
      const;

 private:
  std::unordered_map<ImagePair, TwoViewInfo> twoview_geometries_;
};

inline std::unordered_map<ImagePair, TwoViewInfo>&
ViewGraph::TwoViewGeometries() {
  return twoview_geometries_;
}

inline const std::unordered_map<ImagePair, TwoViewInfo>&
ViewGraph::TwoViewGeometries() const {
  return twoview_geometries_;
}

}  // namespace DAGSfM

#endif