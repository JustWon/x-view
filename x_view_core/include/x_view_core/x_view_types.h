#ifndef X_VIEW_X_VIEW_TYPES_H
#define X_VIEW_X_VIEW_TYPES_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>

#include <memory>

namespace x_view {

// forward declaration
class AbstractSemanticLandmark;
class AbstractLandmarksMatcher;

/// Different types of semantic landmarks to be used in XView
enum SemanticLandmarkType {
  UNDEFINED_SEMANTIC_LANDMARK_TYPE = -1,
  ORB_VISUAL_FEATURE,
  SIFT_VISUAL_FEATURE,
  SURF_VISUAL_FEATURE,
  NUM_SEMANTIC_LANDMARK_TYPES
};

/// Different types of landmarks matchers to be used in XView
enum LandmarksMatcherType {
  UNDEFINED_LANDMARKS_MATCHER_TYPE = -1,
  VECTOR_FEATURES_MATCHER,
  NUM_LANDMARKS_MATCHER_TYPES
};


// typedefs
/// factor graph used for graph optimization
typedef gtsam::NonlinearFactorGraph FactorGraph;

/// 3D pose (position + orientation)
typedef kindr::minimal::QuatTransformationTemplate<double> SE3;

/// pointer to semantic landmark
typedef std::shared_ptr<AbstractSemanticLandmark> SemanticLandmarkPtr;
typedef std::shared_ptr<const AbstractSemanticLandmark>
    ConstSemanticLandmarkPtr;

/// pointer to landmark matchers
typedef std::shared_ptr<AbstractLandmarksMatcher> LandmarksMatcherPtr;
typedef std::shared_ptr<const AbstractLandmarksMatcher>
    ConstLandmarksMatcherPtr;

}

#endif //X_VIEW_X_VIEW_TYPES_H
