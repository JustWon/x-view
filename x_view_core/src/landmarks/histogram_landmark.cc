#include <x_view_core/landmarks/histogram_landmark.h>

namespace x_view {
HistogramLandmark::HistogramLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {


}
}

