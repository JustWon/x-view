#include <x-view_core/x-view_semantics.h>

namespace x_view {

    XViewSemantics::XViewSemantics(const cv::Mat &image, const SE3 &pose)
            : image_(image), pose_(pose) {}

}