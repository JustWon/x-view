#include <gtest/gtest.h>

#include <x_view_core/x_view_types.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark.h>

#include <opencv2/core/core.hpp>
#include <highgui.h>

using namespace x_view;

typedef std::shared_ptr<GraphLandmark> GraphLandmarkPtr;

#define CV_IMAGE_TYPE  CV_8UC1

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

TEST(XViewSlamTestSuite, test_graphLandmark) {

  // Initialize a fake dataset having num_semantic_classes classes
  const int num_semantic_classes = 4;
  globalDatasetPtr =
      std::make_shared<const AbstractDataset>
          (AbstractDataset(num_semantic_classes));

  // create various images
  const int ROWS = 600;
  const int COLS = 1000;

  // image
  cv::Mat blob(ROWS, COLS, CV_IMAGE_TYPE, cv::Scalar(0));
  for (int i = 0; i < ROWS; ++i) {
    for (int j = 0; j < COLS; ++j) {

      std::cout << i << "(" << ROWS/2 << "); " << j << "(" << COLS/2 << ") "
          "--> ";
      if (i < ROWS / 2) {
        if (j >= COLS / 2) {
          blob.at<uchar>(i,j) = (uchar)80;
          std::cout << "80" << std::endl;
        }
      } else {
        if (j < COLS / 2) {
          blob.at<uchar>(i,j) = (uchar)150;
          std::cout << "150" << std::endl;
        } else {
          blob.at<uchar>(i,j) = (uchar)250;
          std::cout << "250" << std::endl;
        }
      }
    }
  }

  std::cout << blob << std::endl;
  cv::imshow("Blob image start", blob);
  cv::waitKey();
}

