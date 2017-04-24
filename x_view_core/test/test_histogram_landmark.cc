#include <gtest/gtest.h>

#include <x_view_core/x_view_types.h>
#include <x_view_core/features/vector_feature.h>
#include <x_view_core/landmarks/histogram_landmark.h>

#include <opencv2/core/core.hpp>

using namespace x_view;

typedef std::shared_ptr<HistogramLandmark> HistogramLandmarkPtr;

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

TEST(XViewSlamTestSuite, histogramLandmark) {
  // create various images
  const int ROWS = 50;
  const int COLS = 30;

  cv::Mat black(ROWS, COLS, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat white(ROWS, COLS, CV_8UC3, cv::Scalar(1, 0, 0));
  cv::Mat half(ROWS, COLS, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < ROWS / 2; ++i) {
    for (int j = 0; j < COLS; ++j) {
      half.at<cv::Vec3b>(i, j) = cv::Vec3b(1, 0, 0);
    }
  }

  SemanticLandmarkPtr bLandmark_ = HistogramLandmark::create(black, SE3());
  HistogramLandmarkPtr bLandmark = CAST(bLandmark_, HistogramLandmark);

  SemanticLandmarkPtr wLandmark_ = HistogramLandmark::create(white, SE3());
  HistogramLandmarkPtr wLandmark = CAST(wLandmark_, HistogramLandmark);

  SemanticLandmarkPtr hLandmark_ = HistogramLandmark::create(half, SE3());
  HistogramLandmarkPtr hLandmark = CAST(hLandmark_, HistogramLandmark);

  cv::Mat bHist =
      CAST(bLandmark->getFeature(), const VectorFeature)->getFeature();
  cv::Mat wHist =
      CAST(wLandmark->getFeature(), const VectorFeature)->getFeature();
  cv::Mat hHist =
      CAST(hLandmark->getFeature(), const VectorFeature)->getFeature();

  auto toVec = [](const cv::Mat& mat) -> std::vector<float> {
    std::vector<float> v(mat.cols, 0.f);
    for (int i = 0; i < mat.cols; ++i) {
      v[i] = mat.at<float>(i);
    }
    return v;
  };

  auto bVec = toVec(bHist);
  auto wVec = toVec(wHist);
  auto hVec = toVec(hHist);

  CHECK_DOUBLE_EQ(bVec[0], 1.0);
  CHECK_DOUBLE_EQ(wVec[1], 1.0);
  CHECK_DOUBLE_EQ(hVec[0], hVec[1]);
  CHECK_DOUBLE_EQ(hVec[0], 0.5);


}

