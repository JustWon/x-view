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

/**
 * \brief Counts how many pixels with each label are present in the image
 * \param image cv::Mat to be analyzed
 * \param pixelCount vector filled up with the pixel count such that
 * 'pixelCount[i]' contains the number of pixels in 'image' having label 'i'
 */
void countPixelLabelsInImage(const cv::Mat& image,
                             std::vector<int>& pixelCount);

/**
 * \brief Checks if the number of pixels for each class 'i' in 'image' is the
 * same between counting them explicitly and the one contained in the
 * graphLandmarkPointer
 * \param graphLandmarkPtr pointer to the graphLandmark
 * \param imageName logging image name
 */
void testPixelCount(const GraphLandmarkPtr& graphLandmarkPtr,
                    const std::string& imageName);

void testInstanceCount(const GraphLandmarkPtr& graphLandmarkPtr,
                       const std::vector<int>& instanceCount,
                       const std::string& imageName);

void createCustomImage(const int desiredRows, const int desiredCols,
                       cv::Mat& image);

void createChessBoardImage(const int desiredRows, const int desiredCols,
                           const int block_size, cv::Mat& image);

void testCustomImage();
void testChessboardImage();

#define PRINT_LINE std::cout << "at line: " << __LINE__ << std::endl;

TEST(XViewSlamTestSuite, test_graphLandmark) {

  // test different images
  PRINT_LINE
  testCustomImage();
  PRINT_LINE
  testChessboardImage();
  PRINT_LINE
}

void testCustomImage() {
  // Initialize a fake dataset having num_semantic_classes classes
  int num_semantic_classes = 4;
  globalDatasetPtr =
      std::make_shared<const AbstractDataset>
          (AbstractDataset(num_semantic_classes));

  const int rows = 500;
  const int cols = 700;

  cv::Mat customImage;
  PRINT_LINE
  createCustomImage(rows, cols, customImage);
  PRINT_LINE
  GraphLandmarkPtr customImageLandmarkPtr =
      CAST(GraphLandmark::create(customImage, SE3()), GraphLandmark);
  PRINT_LINE
  testPixelCount(customImageLandmarkPtr, "customImage");
  PRINT_LINE
  testInstanceCount(customImageLandmarkPtr, {1, 1, 1, 1}, "customImage");
  PRINT_LINE
}
void testChessboardImage() {
  // Initialize a fake dataset having num_semantic_classes classes
  int num_semantic_classes = 3;
  globalDatasetPtr =
      std::make_shared<const AbstractDataset>
          (AbstractDataset(num_semantic_classes));

  const int rows = 600;
  const int cols = 1200;
  const int block_size = 40;

  cv::Mat chessboard;
  PRINT_LINE
  createChessBoardImage(rows, cols, block_size, chessboard);
  PRINT_LINE

  GraphLandmarkPtr chessImageLandmarkPtr =
      CAST(GraphLandmark::create(chessboard, SE3()), GraphLandmark);
  PRINT_LINE
  testPixelCount(chessImageLandmarkPtr, "chessboardImage");
  PRINT_LINE

  auto computeInstanceCount = [&](std::vector<int>& instanceCount) -> void {

    // compute the new number of cols and rows
    auto nearestMultipleOf = [](const int multiple, const int val) -> int {
      return multiple * ((int) ((float) val / multiple));
    };

    const int newRows = rows / block_size * block_size;
    const int newCols =
        nearestMultipleOf(globalDatasetPtr->numSemanticClasses(),
                          cols / block_size) * block_size;

    // compute number of 'chess' blocks
    const int nBlocksRows = newRows / block_size;
    const int nBlocksCols = newCols / block_size;
    const int nBlocks = nBlocksRows * nBlocksCols;

    // assign the instance count by looping through the blocks
    instanceCount.resize(globalDatasetPtr->numSemanticClasses());
    int currentLabelIndex = 0;
    for (int block = 0; block < nBlocks; ++block) {
      instanceCount[currentLabelIndex]++;
      currentLabelIndex = (currentLabelIndex + 1)
          % globalDatasetPtr->numSemanticClasses();
    }

    for(int i = 0; i < instanceCount.size(); ++i) {
      std::cout << "Number of instances for class " << i << " is: " <<
                                                                    instanceCount[i] << std::endl;
    }
  };

  PRINT_LINE
  std::vector<int> instanceCount;
  computeInstanceCount(instanceCount);
  PRINT_LINE
  testInstanceCount(chessImageLandmarkPtr, instanceCount, "chessBoardImage");
  PRINT_LINE
}

void countPixelLabelsInImage(const cv::Mat& image,
                             std::vector<int>& pixelCount) {
  pixelCount.clear();
  pixelCount.resize(globalDatasetPtr->numSemanticClasses());
  for (int i = 0; i < image.rows; ++i) {
    for (int j = 0; j < image.cols; ++j) {
      int label = static_cast<int>(image.at<cv::Vec3b>(i, j)[0]);
      pixelCount[label]++;
    }
  }
}

void testPixelCount(const GraphLandmarkPtr& graphLandmarkPtr,
                    const std::string& imageName) {

  // vector counting explicitly the number of pixels
  std::vector<int> expectedPixelCount;
  countPixelLabelsInImage(graphLandmarkPtr->getSemanticImage(),
                          expectedPixelCount);

  for (int i = 0; i < expectedPixelCount.size(); ++i) {
    int instancePixelCount = 0;
    auto const& instanceBlobs = graphLandmarkPtr->getBlobs()[i];
    for (int j = 0; j < instanceBlobs.size(); ++j)
      instancePixelCount += instanceBlobs[j].size();
    CHECK_EQ(expectedPixelCount[i], instancePixelCount)
      << "In image " << imageName << ", class instance " << i
      << " should have " << expectedPixelCount[i] << " pixels, but has "
      << instancePixelCount;
  }
}

void testInstanceCount(const GraphLandmarkPtr& graphLandmarkPtr,
                       const std::vector<int>& expectedInstanceCount,
                       const std::string& imageName) {
  auto const& blobs = graphLandmarkPtr->getBlobs();
  for (int i = 0; i < expectedInstanceCount.size(); ++i) {
    CHECK_EQ(expectedInstanceCount[i], blobs[i].size())
      << "In image " << imageName << ", class " << i << " should have "
      << expectedInstanceCount[i] << " instances, but has " << blobs[i].size();
  }
}

void createCustomImage(const int desiredRows, const int desiredCols,
                       cv::Mat& image) {

  image.create(desiredRows, desiredCols, CV_IMAGE_TYPE);
  for (int i = 0; i < desiredRows; ++i) {
    for (int j = 0; j < desiredCols; ++j) {
      if (i < desiredRows / 2) {
        if (j < desiredCols / 2) {
          image.at<cv::Vec3b>(i, j)[0] = (uchar) 0;
        } else {
          image.at<cv::Vec3b>(i, j)[0] = (uchar) 1;
        }
      } else {
        if (j < desiredCols / 2) {
          image.at<cv::Vec3b>(i, j)[0] = (uchar) 2;
        } else {
          image.at<cv::Vec3b>(i, j)[0] = (uchar) 3;
        }
      }
    }
  }
}

void createChessBoardImage(const int desiredRows, const int desiredCols,
                           const int block_size, cv::Mat& image) {

  auto nearestMultipleOf = [](const int multiple, const int val) -> int {
    return multiple * ((int) ((float) val / multiple));
  };

  const int newRows = desiredRows / block_size * block_size;
  const int newCols =
      nearestMultipleOf(globalDatasetPtr->numSemanticClasses(),
                        desiredCols / block_size) * block_size;

  image.create(newRows, newCols, CV_IMAGE_TYPE);

  unsigned char color = 0;
  for (int i = 0; i < newRows; i = i + block_size) {
    color = (uchar) ((color + 1) % globalDatasetPtr->numSemanticClasses());
    for (int j = 0; j < newCols; j = j + block_size) {
      cv::Mat ROI = image(cv::Rect(j, i, block_size, block_size));
      ROI.setTo(cv::Scalar(color, 0, 0));
      color =
          (uchar) (((int) color + 1) % globalDatasetPtr->numSemanticClasses());
    }
  }
}

