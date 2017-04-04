#ifndef X_VIEW_SEMANTIC_GRAPH_H_
#define  X_VIEW_SEMANTIC_GRAPH_H_

#include <x-view_core/types.h>
#include <x-view_core/x-view_semantics.h>

#include <opencv2/core/core.hpp>

namespace x_view {

    struct SemanticGraph : public XViewSemantics {

        explicit SemanticGraph(const cv::Mat &image, const SE3& pose);

        virtual XViewSemanticMatchingResult match(const XViewSemantics& other);

    };
}

#endif // X_VIEW_SEMANTIC_GRAPH_H_