#ifndef X_VIEW_BOS_H_
#define  X_VIEW_BOS_H_

#include <x-view_core/x-view_types.h>
#include <x-view_core/landmarks/abstract_semantic_landmark.h>

#include <opencv2/core/core.hpp>

namespace x_view {

    struct BoS : public AbstractSemanticLandmark {

        explicit BoS(const cv::Mat &image, const SE3& pose);

        virtual XViewSemanticMatchingResult match(const AbstractSemanticLandmark& other);

    };
}

#endif // X_VIEW_BOS_H_