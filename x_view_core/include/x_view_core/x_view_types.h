#ifndef X_VIEW_X_VIEW_TYPES_H
#define X_VIEW_X_VIEW_TYPES_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>

#include <memory>

namespace x_view {

// forward declaration
struct AbstractSemanticLandmark;

// typedefs
/// factor graph used for graph optimization
typedef gtsam::NonlinearFactorGraph FactorGraph;

/// 3D pose (position + orientation)
typedef kindr::minimal::QuatTransformationTemplate<double> SE3;

/// pointer to semantic landmark
typedef std::shared_ptr<AbstractSemanticLandmark> SemanticLandmarkPtr;

}

#endif //X_VIEW_X_VIEW_TYPES_H
