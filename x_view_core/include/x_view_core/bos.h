#ifndef X_VIEW_BOS_H_
#define  X_VIEW_BOS_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/abstract_semantic_landmark.h>

#include <opencv2/core/core.hpp>

namespace x_view {

struct BoS : public AbstractSemanticLandmark {

  explicit BoS(const cv::Mat& image, const SE3& pose);

  virtual SemanticMatchingResult match(const AbstractSemanticLandmark& other);

};
}

#endif // X_VIEW_BOS_H_
