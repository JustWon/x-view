#ifndef X_VIEW_BOS_H_
#define  X_VIEW_BOS_H_

#include <x-view_core/types.h>
#include <x-view_core/x-view_semantics.h>

#include <opencv2/core/core.hpp>

namespace x_view {

    struct BoS : public XViewSemantics {

        explicit BoS(const cv::Mat &image, const SE3& pose);

    };
}

#endif // X_VIEW_BOS_H_