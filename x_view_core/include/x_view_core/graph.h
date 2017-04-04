#ifndef X_VIEW_GRAPH_H_
#define  X_VIEW_GRAPH_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/abstract_semantic_landmark.h>

#include <opencv2/core/core.hpp>

namespace x_view {

struct SemanticGraph : public AbstractSemanticLandmark {

  explicit SemanticGraph(const cv::Mat& image, const SE3& pose);

  virtual SemanticMatchingResult match(const AbstractSemanticLandmark& other);

};
}

#endif // X_VIEW_GRAPH_H_
