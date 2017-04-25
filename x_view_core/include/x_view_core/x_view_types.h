#ifndef X_VIEW_X_VIEW_TYPES_H
#define X_VIEW_X_VIEW_TYPES_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>

#include <memory>

namespace x_view {

// forward declaration
class AbstractDataset;
class AbstractDescriptor;
class AbstractSemanticLandmark;
class AbstractMatcher;

/// Different types of feature representation used in XView
enum FeatureType {
  UNDEFINED_FEATURE_TYPE = -1,
  VECTOR_FEATURE,
  GRAPH_FEATURE,
  NUM_FEATURE_TYPES
};

/// Different types of semantic landmarks to be used in XView
enum SemanticLandmarkType {
  UNDEFINED_SEMANTIC_LANDMARK_TYPE = -1,
  ORB_VISUAL_FEATURE,
  SIFT_VISUAL_FEATURE,
  SURF_VISUAL_FEATURE,
  SEMANTIC_HISTOGRAM,
  NUM_SEMANTIC_LANDMARK_TYPES
};

/// Different types of landmarks matchers to be used in XView
enum LandmarksMatcherType {
  UNDEFINED_LANDMARKS_MATCHER_TYPE = -1,
  VECTOR_MATCHER,
  NUM_LANDMARKS_MATCHER_TYPES
};

// typedefs
/// factor graph used for graph optimization
typedef gtsam::NonlinearFactorGraph FactorGraph;

/// 3D pose (position + orientation)
typedef kindr::minimal::QuatTransformationTemplate<double> SE3;

/// pointer to dataset object
typedef std::shared_ptr<AbstractDataset> DatasetPtr;

typedef std::shared_ptr<const AbstractDataset> ConstDatasetPrt;

/// pointer to feature
typedef std::shared_ptr<AbstractDescriptor> DescriptorPtr;

typedef std::shared_ptr<const AbstractDescriptor> ConstDescriptorPtr;

/// pointer to semantic landmark
typedef std::shared_ptr<AbstractSemanticLandmark> SemanticLandmarkPtr;

typedef std::shared_ptr<const AbstractSemanticLandmark>
    ConstSemanticLandmarkPtr;

/// pointer to landmark matchers
typedef std::shared_ptr<AbstractMatcher> LandmarksMatcherPtr;

typedef std::shared_ptr<const AbstractMatcher>
    ConstLandmarksMatcherPtr;

}

#endif //X_VIEW_X_VIEW_TYPES_H
