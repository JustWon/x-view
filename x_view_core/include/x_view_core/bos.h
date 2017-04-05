#ifndef X_VIEW_BOS_H_
#define  X_VIEW_BOS_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/abstract_semantic_landmark.h>

#include <opencv2/core/core.hpp>

#include <vector>

namespace x_view {

struct BoS : public AbstractSemanticLandmark {

  /**
   * \brief depending on the dataset we are working on, the possible number
   * of different extracted semantic entities varies. In particular, using
   * the Synthia dataset, we have 13 classes
   */
  static int NUM_SEMANTIC_ENTITIES;

  explicit BoS(const cv::Mat& image, const SE3& pose);

  virtual SemanticMatchingResult match(const AbstractSemanticLandmark& other);


  std::vector<int> semantic_entity_count_;

};
}

#endif // X_VIEW_BOS_H_
