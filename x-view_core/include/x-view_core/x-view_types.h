#ifndef X_VIEW_X_VIEW_TYPES_H
#define X_VIEW_X_VIEW_TYPES_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>


namespace x_view {

    typedef gtsam::NonlinearFactorGraph FactorGraph;
    typedef kindr::minimal::QuatTransformation SE3;
}

#endif //X_VIEW_X_VIEW_TYPES_H
