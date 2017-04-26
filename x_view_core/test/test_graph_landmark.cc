#include <gtest/gtest.h>

#include <x_view_core/x_view_types.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark.h>

#include <opencv2/core/core.hpp>
#include <highgui.h>

using namespace x_view;

typedef std::shared_ptr<GraphLandmark> GraphLandmarkPtr;

#define CV_IMAGE_TYPE  CV_8UC3

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

cv::Mat createChessBoardImage(const int desiredRows, const int desiredCols,
                              const int BLOCK_SIZE) {

  auto nearestMultipleOf = [](const int multiple, const int val) -> int {
    return multiple * ((int) ((float) val / multiple));
  };

  const int newRows = desiredRows / BLOCK_SIZE * BLOCK_SIZE;
  const int newCols =
      nearestMultipleOf(globalDatasetPtr->numSemanticClasses(),
                        desiredCols / BLOCK_SIZE) * BLOCK_SIZE;

  cv::Mat chessBoard(newRows, newCols, CV_IMAGE_TYPE, cv::Scalar::all(0));

  unsigned char color = 0;
  for (int i = 0; i < newRows; i = i + BLOCK_SIZE) {
    color = (uchar) ((color + 1) % globalDatasetPtr->numSemanticClasses());
    for (int j = 0; j < newCols; j = j + BLOCK_SIZE) {
      cv::Mat ROI = chessBoard(cv::Rect(j, i, BLOCK_SIZE, BLOCK_SIZE));
      ROI.setTo(cv::Scalar(color,0,0));
      color =
          (uchar) (((int) color + 1) % globalDatasetPtr->numSemanticClasses());
    }
  }
  return chessBoard;
}

TEST(XViewSlamTestSuite, test_graphLandmark) {

  // Initialize a fake dataset having num_semantic_classes classes
  int num_semantic_classes = 4;
  globalDatasetPtr =
      std::make_shared<const AbstractDataset>
          (AbstractDataset(num_semantic_classes));

  // create various images of this size
  const int ROWS = 60;
  const int COLS = 120;

  // Custom Image
  cv::Mat blob(ROWS, COLS, CV_IMAGE_TYPE, cv::Scalar(0, 0, 0));
  for (int i = 0; i < ROWS; ++i) {
    for (int j = 0; j < COLS; ++j) {
      if (i < ROWS / 2) {
        if (j >= COLS / 2) {
          blob.at<cv::Vec3b>(i, j)[0] = (uchar) 1;
        }
      } else {
        if (j < COLS / 2) {
          blob.at<cv::Vec3b>(i, j)[0] = (uchar) 2;
        } else {
          blob.at<cv::Vec3b>(i, j)[0] = (uchar) 0;

        }
      }
    }
  }
  GraphLandmarkPtr landmarkPtr =
      CAST(GraphLandmark::create(blob, SE3()), GraphLandmark);

  std::cout << "GraphLandmark custom created" << std::endl;

  // Chessboard image
  const int BLOCK_SIZE = 12;;
  cv::Mat chessboard = createChessBoardImage(ROWS, COLS, BLOCK_SIZE);
  GraphLandmarkPtr chessLandmark =
      CAST(GraphLandmark::create(chessboard, SE3()), GraphLandmark);

  std::cout << "GraphLandmark chessboard created" << std::endl;


}

