/* This test computes the histogram representation of three different images,
 * namely a black image, a white image and a half-black half-white image.
 * The test verifies that the HistogramLandmark class correctly extracts a
 * histogram representation of the image, i.e. it checks that the resulting
 * histograms contain the correct percentage of label '0' (black) and label
 * '1' (white).
 */

#include <gtest/gtest.h>
#include "test_histogram_landmark.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/histogram_landmark.h>
#include <x_view_core/x_view_locator.h>

using namespace x_view;
using namespace x_view_test;

#define CV_IMAGE_TYPE  CV_8UC3

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

TEST(XViewSlamTestSuite, test_histogram_landmark) {

  LOG(INFO) << "\n\n====Testing histogram landmark====";

  // Initialize a fake dataset having num_semantic_classes classes
  const int num_semantic_classes = 2;
  std::unique_ptr<AbstractDataset> dataset(
      new AbstractDataset(num_semantic_classes));
  Locator::registerDataset(std::move(dataset));

  // create various images
  const int ROWS = 50;
  const int COLS = 30;
  const unsigned long frame_index = 0;

  // Black image
  cv::Mat black(ROWS, COLS, CV_IMAGE_TYPE, cv::Scalar(0, 0, 0));
  FrameData frame_data_black(black, cv::Mat(), SE3(), frame_index);
  SemanticLandmarkPtr bLandmark = HistogramLandmark::create(frame_data_black);
  // expect to have 100% votes for label 0
  std::vector<std::pair<int, double>> bExpected = {
      {0, 1}, {1, 0}
  };
  performLabelTest(bLandmark, bExpected);

  // White image
  cv::Mat white(ROWS, COLS, CV_IMAGE_TYPE, cv::Scalar(1, 0, 0));
  FrameData frame_data_white(white, cv::Mat(), SE3(), frame_index);
  SemanticLandmarkPtr wLandmark = HistogramLandmark::create(frame_data_white);
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
  FrameData frame_data_half(half, cv::Mat(), SE3(), frame_index);
  SemanticLandmarkPtr hLandmark = HistogramLandmark::create(frame_data_half);
  // expect to have 50% votes for label 0 and 50% for label 1
  std::vector<std::pair<int, double>> hExpected = {
      {0, 0.5}, {1, 0.5}, {2, 0}
  };
  performLabelTest(hLandmark, hExpected);

}

