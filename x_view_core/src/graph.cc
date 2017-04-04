#include <x_view_core/graph.h>

#include <glog/logging.h>

namespace x_view {

SemanticGraph::SemanticGraph(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {
  CHECK(false) << "Not implemented yet";
}

SemanticMatchingResult SemanticGraph::match(const AbstractSemanticLandmark& other) {
  CHECK(false) << "Not implemented yet";
}
}
