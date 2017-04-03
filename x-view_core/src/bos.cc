#include <x-view_core/bos.h>

#include <glog/logging.h>

namespace x_view {

    BoS::BoS(const cv::Mat &image, const SE3& pose) : XViewSemantics(image, pose) {
        CHECK(false) << "Not implemented yet";
    }
}

