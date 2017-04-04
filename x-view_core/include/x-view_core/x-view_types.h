#ifndef X_VIEW_X_VIEW_TYPES_H
#define X_VIEW_X_VIEW_TYPES_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>

#include <memory>


namespace x_view {

    // forward declaration
    struct AbstractSemanticLandmark;


    // typedefs
    typedef gtsam::NonlinearFactorGraph FactorGraph;                  /// factor graph used for graph optimization
    typedef kindr::minimal::QuatTransformationTemplate<double> SE3;   /// 3D pose (position + orientation)

    typedef std::shared_ptr<AbstractSemanticLandmark> SemanticLandmarkPtr;   /// pointer to semantic landmark
}

#endif //X_VIEW_X_VIEW_TYPES_H
