#include <highgui.h>
#include "test_graph_landmark_impl.h"

#define CV_IMAGE_TYPE  CV_8UC3

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

void testCustomImage() {

  const std::string image_name = "custom image\n";

  // Initialize a fake dataset having num_semantic_classes classes
  int num_semantic_classes = 4;
  global_dataset_ptr =
      std::make_shared<const AbstractDataset>
          (AbstractDataset(num_semantic_classes));

#ifdef X_VIEW_DEBUG
  std::vector<int> sizes = {100, 200, 333};
#else
  std::vector<int> sizes = {100, 200, 333, 800, 1500};
#endif // X_VIEW_DEBUG

  boost::progress_display show_progress(sizes.size(), std::cout, image_name);
  for (auto size : sizes) {
    const int rows = size;
    const int cols = static_cast<int>(size * 1.5);

    cv::Mat customImage;
    createCustomImage(rows, cols, customImage);
    GraphLandmarkPtr customImageLandmarkPtr =
        CAST(GraphLandmark::create(customImage, SE3()), GraphLandmark);

    testPixelCount(customImageLandmarkPtr, image_name);
    testInstanceCount(customImageLandmarkPtr, {1, 1, 1, 1}, image_name);

    ++show_progress;
  }
}

void testDiscImage() {

  const std::string image_name = "disc image\n";
  // random number generator to generate discs
  cv::RNG rng(2);

  std::vector<int> classes = {2, 3, 5, 15};
  std::vector<int> num_discs = {2, 15};
  boost::progress_display show_progress(classes.size() * num_discs.size(),
                                        std::cout,
                                        image_name);
  for (auto numClasses : classes) {
    global_dataset_ptr =
        std::make_shared<const AbstractDataset>
            (AbstractDataset(numClasses));

    const int rows = 500;
    const int cols = static_cast<int>(500 * 1.5);
    const int diag = static_cast<int>(std::sqrt(rows * rows + cols * cols));
    for (int num_disc : num_discs) {

      cv::Mat discImage;
      std::vector<cv::Point> centers;
      std::vector<int> radii, labels;

      for (int i = 0; i < num_disc; ++i) {
        centers.push_back(cv::Point(rng.uniform(0, cols),
                                    rng.uniform(0, rows)));
        radii.push_back(std::max(15, rng.uniform(diag / 40, diag / 10)));
        labels.push_back(rng.uniform(1, numClasses));
      }

      createDiscImage(rows, cols, centers, radii, labels, discImage);

      GraphLandmarkPtr discImageLandmarkPtr =
          CAST(GraphLandmark::create(discImage, SE3()), GraphLandmark);

      testPixelCount(discImageLandmarkPtr, image_name);

      ++show_progress;
    }

  }

}

void countPixelLabelsInImage(const cv::Mat& image,
                             std::vector<int>& pixelCount) {
  pixelCount.clear();
  pixelCount.resize(global_dataset_ptr->numSemanticClasses());
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
      instancePixelCount += instanceBlobs[j].num_pixels_;
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
      << expectedInstanceCount[i] << " instances, but has "
      << blobs[i].size();
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

void createDiscImage(const int desiredRows, const int desiredCols,
                     const std::vector<cv::Point>& centers,
                     const std::vector<int> radii,
                     const std::vector<int> labels, cv::Mat& image) {

  image.create(desiredRows, desiredCols, CV_IMAGE_TYPE);
  image = cv::Scalar::all(0);

  for (int c = 0; c < centers.size(); ++c)
    cv::circle(image, centers[c], radii[c], cv::Scalar(labels[c], 0, 0),
               -1, 8, 0);

}
