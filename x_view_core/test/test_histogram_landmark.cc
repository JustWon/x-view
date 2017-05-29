/* This test computes the histogram representation of three different images,
 * namely a black image, a white image and a half-black half-white image.
 * The test verifies that the HistogramLandmark class correctly extracts a
 * histogram representation of the image, i.e. it checks that the resulting
 * histograms contain the correct percentage of label '0' (black) and label
 * '1' (white).
 */

#include <gtest/gtest.h>

#include <x_view_core/x_view_types.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/vector_descriptor.h>
#include <x_view_core/landmarks/histogram_landmark.h>

#include <opencv2/core/core.hpp>

using namespace x_view;

typedef std::shared_ptr<HistogramLandmark> HistogramLandmarkPtr;

#define CV_IMAGE_TYPE  CV_8UC3

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

TEST(XViewSlamTestSuite, test_histogram_landmark) {

  // Initialize a fake dataset having num_semantic_classes classes
  const int num_semantic_classes = 2;
  global_dataset_ptr =
      std::make_shared<const AbstractDataset>
          (AbstractDataset(num_semantic_classes));

  // create various images
  const int ROWS = 50;
  const int COLS = 30;

  // Black image
  cv::Mat black(ROWS, COLS, CV_IMAGE_TYPE, cv::Scalar(0, 0, 0));
  SemanticLandmarkPtr bLandmark = HistogramLandmark::create(black, SE3());
  // expect to have 100% votes for label 0
  std::vector<std::pair<int, double>> bExpected = {
      {0, 1}, {1, 0}
  };
  performLabelTest(bLandmark, bExpected);

  // White image
  cv::Mat white(ROWS, COLS, CV_IMAGE_TYPE, cv::Scalar(1, 0, 0));
  SemanticLandmarkPtr wLandmark = HistogramLandmark::create(white, SE3());
  // expect to have 100% votes in label 1
  std::vector<std::pair<int, double>> wExpected = {
      {0, 0}, {1, 1}
  };
  performLabelTest(wLandmark, wExpected);


  // Half white half black image
  cv::Mat half(ROWS, COLS, CV_IMAGE_TYPE, cv::Scalar(0, 0, 0));
  for (int i = 0; i < ROWS / 2; ++i) {
    for (int j = 0; j < COLS; ++j) {
      half.at<cv::Vec3b>(i, j) = cv::Vec3b(1, 0, 0);
    }
  }
  SemanticLandmarkPtr hLandmark = HistogramLandmark::create(half, SE3());
  // expect to have 50% votes for label 0 and 50% for label 1
  std::vector<std::pair<int, double>> hExpected = {
      {0, 0.5}, {1, 0.5}, {2, 0}
  };
  performLabelTest(hLandmark, hExpected);

}
