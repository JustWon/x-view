#include <gtest/gtest.h>

#include <x_view_core/x_view_types.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark.h>

#include <opencv2/core/core.hpp>
#include <highgui.h>

#include <chrono>

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

/**
 * \brief Checks if the number of instances per class found by the
 * graphLandmarkPrt object is the same as the expected one
 * \param graphLandmarkPtr pointer to the graphLandmark
 * \param expectedInstanceCount expected number of instances per class, such
 * that 'expectedInstanceCount[i]' contains the number of expected instances
 * for class 'i'
 * \param imageName logging image name
 */
void testInstanceCount(const GraphLandmarkPtr& graphLandmarkPtr,
                       const std::vector<int>& expectedInstanceCount,
                       const std::string& imageName);

/**
 * \brief Creates a custom image of size 'desiredRows' x 'desiredCols'
 * \param desiredRows desired number of rows
 * \param desiredCols desired number of cols
 * \param image generated image
 */
void createCustomImage(const int desiredRows, const int desiredCols,
                       cv::Mat& image);

/**
 * \brief Creates a chessboard-like image of approximate size 'desiredRows' x
 * 'desiredCols' with a block size of 'block_size'. The generated image
 * represents a sets of blocks in the following order:
 * 0, 1, 2, 3, ... , N-1, 0, 1, 2, 3, ..., N-1, ...., N-1
 * 1, 2, 3, 4, ... , 0, 1, 2, 3, 4, ...
 * ...
 * where 'N' is equal to 'globalDatasetPtr->numSemanticClasses()'.
 * For this reason the final image size might not be exactly the same as the
 * one passed as parameter
 * \param desiredRows desired number of rows
 * \param desiredCols desired number of cols
 * \param block_size size of the chessboard block
 * \param image generated image
 */
void createChessBoardImage(const int desiredRows, const int desiredCols,
                           const int block_size, cv::Mat& image);

/// \brief test the custom image
void testCustomImage();
/// \brief test the chessboard image
void testChessboardImage();

TEST(XViewSlamTestSuite, test_graphLandmark) {

  // test different images
  testCustomImage();
  testChessboardImage();
}

void testCustomImage() {
  // Initialize a fake dataset having num_semantic_classes classes
  int num_semantic_classes = 4;
  globalDatasetPtr =
      std::make_shared<const AbstractDataset>
          (AbstractDataset(num_semantic_classes));

  std::vector<int> sizes = {100, 200, 333, 800, 1500};
  for (auto size : sizes) {
    std::cout << "Testing customImage for size " << size << " ... ";
    const int rows = size;
    const int cols = static_cast<int>(size * 1.5);

    cv::Mat customImage;
    createCustomImage(rows, cols, customImage);
    auto t1 = std::chrono::high_resolution_clock::now();
    GraphLandmarkPtr customImageLandmarkPtr =
        CAST(GraphLandmark::create(customImage, SE3()), GraphLandmark);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto timespan =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    testPixelCount(customImageLandmarkPtr, "customImage");
    testInstanceCount(customImageLandmarkPtr, {1, 1, 1, 1}, "customImage");
    std::cout << " completed in " << timespan.count() << " s. !" << std::endl;
  }
}
void testChessboardImage() {
  // Initialize a fake dataset having num_semantic_classes classes
  std::vector<int> classes = {2, 3, 5, 15};
  for (auto numClasses : classes) {
    int num_semantic_classes = numClasses;
    globalDatasetPtr =
        std::make_shared<const AbstractDataset>
            (AbstractDataset(num_semantic_classes));

    std::vector<int> sizes = {100, 200, 333, 800};
    for (auto size : sizes) {
      const int rows = size;
      const int cols = static_cast<int>(size * 1.5);

      std::vector<int> block_sizes = {20, 21, 40};
      for (auto block_size : block_sizes) {

        std::cout << "Testing chessboardImage for " << num_semantic_classes
                  << " semantic classes, size " << size << " and block size "
                  << block_size << " ... ";

        cv::Mat chessboard;
        createChessBoardImage(rows, cols, block_size, chessboard);

        auto t1 = std::chrono::high_resolution_clock::now();
        GraphLandmarkPtr chessImageLandmarkPtr =
            CAST(GraphLandmark::create(chessboard, SE3()), GraphLandmark);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto timespan =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        testPixelCount(chessImageLandmarkPtr, "chessboardImage");

        auto computeInstanceCount =
            [&](std::vector<int>& instanceCount) -> void {

              // compute the new number of cols and rows
              auto nearestMultipleOf =
                  [](const int multiple, const int val) -> int {
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
            };

        std::vector<int> instanceCount;
        computeInstanceCount(instanceCount);

        testInstanceCount(chessImageLandmarkPtr,
                          instanceCount,
                          "chessBoardImage");

        std::cout << " completed in " << timespan.count()
                  << " s. !" << std::endl;

      }
    }
  }
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

