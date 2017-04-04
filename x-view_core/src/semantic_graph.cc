#include <x-view_core/semantic_graph.h>

#include <glog/logging.h>

namespace x_view {

    SemanticGraph::SemanticGraph(const cv::Mat &image, const SE3& pose) : XViewSemantics(image, pose) {
        CHECK(false) << "Not implemented yet";
    }

    XViewSemanticMatchingResult SemanticGraph::match(const XViewSemantics& other) {
        CHECK(false) << "Not implemented yet";
    }
}

