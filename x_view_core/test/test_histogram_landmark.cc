#include "test_histogram_landmark.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/vector_descriptor.h>
#include <x_view_core/landmarks/histogram_landmark.h>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

using namespace x_view;

namespace x_view_test {

typedef std::shared_ptr<HistogramLandmark> HistogramLandmarkPtr;

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

void performLabelTest(const SemanticLandmarkPtr& landmark_, const
std::vector<std::pair<int, double>>& expected) {

  HistogramLandmarkPtr landmark = CAST(landmark_, HistogramLandmark);

  cv::Mat hist =
      CAST(landmark->getDescriptor(), const VectorDescriptor)->getDescriptor();

  auto toVec = [](const cv::Mat& mat) -> std::vector<float> {
    std::vector<float> v(mat.cols, 0.f);
    for (int i = 0; i < mat.cols; ++i) {
      v[i] = mat.at<float>(i);
    }
    return v;
  };

  auto vec = toVec(hist);

  for (int condition = 0; condition < expected.size(); ++condition) {
    const int binIndex = expected[condition].first;
    const double expectedPercentage = expected[condition].second;
    CHECK_DOUBLE_EQ(vec[binIndex], expectedPercentage);
  }
}

}
