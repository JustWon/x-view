#include <x-view_core/landmarks/bos.h>

#include <glog/logging.h>

namespace x_view {

    BoS::BoS(const cv::Mat &image, const SE3& pose) : AbstractSemanticLandmark(image, pose) {
        CHECK(false) << "Not implemented yet";
    }

    XViewSemanticMatchingResult BoS::match(const AbstractSemanticLandmark& other) {
        CHECK(false) << "Not implemented yet";
    }
}

