#include "test_graph_landmark.h"

#include <glog/logging.h>

namespace x_view_test {

#define CV_IMAGE_TYPE  CV_8UC3

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

void testCustomImage() {

  const std::string image_name = "custom image";

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

  for (auto size : sizes) {
    const int rows = size;
    const int cols = static_cast<int>(size * 1.5);
    LOG(INFO) << "Testing " << image_name << " for image size: ["
              << rows << " x " << cols << "]";

    cv::Mat customImage;
    createCustomImage(rows, cols, customImage);
    GraphLandmarkPtr customImageLandmarkPtr =
        CAST(GraphLandmark::create(customImage, SE3()), GraphLandmark);

    CHECK_NOTNULL(customImageLandmarkPtr.get());

    testPixelCount(customImageLandmarkPtr, image_name);
    testBlobsCount(customImageLandmarkPtr, {1, 1, 1, 1}, image_name);
    LOG(INFO) << "Test passed.";
  }
}

void testDiscImage() {

  const std::string image_name = "disc image";
  // random number generator to generate discs
  cv::RNG rng(2);

  std::vector<int> classes = {2, 3, 5, 15};
  std::vector<int> num_discs = {2, 15};

  for (auto numClasses : classes) {
    global_dataset_ptr =
        std::make_shared<const AbstractDataset>
            (AbstractDataset(numClasses));

    const int rows = 500;
    const int cols = static_cast<int>(500 * 1.5);
    const int diag = static_cast<int>(std::sqrt(rows * rows + cols * cols));
    for (int num_disc : num_discs) {
      LOG(INFO) << "Testing " << image_name << " for image size: ["
                << rows << " x " << cols << "] and " << num_disc << " discs.";

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
      LOG(INFO) << "Test passed.";
    }

  }

}

void countPixelLabelsInImage(const cv::Mat& image,
                             std::vector<int>& pixel_count) {
  pixel_count.clear();
  pixel_count.resize(global_dataset_ptr->numSemanticClasses());
  for (int i = 0; i < image.rows; ++i) {
    for (int j = 0; j < image.cols; ++j) {
      int label = static_cast<int>(image.at<cv::Vec3b>(i, j)[0]);
      pixel_count[label]++;
    }
  }
}

void testPixelCount(const GraphLandmarkPtr& graphLandmarkPtr,
                    const std::string& imageName) {

  // vector counting explicitly the number of pixels
  std::vector<int> expected_pixel_count;
  countPixelLabelsInImage(graphLandmarkPtr->getSemanticImage(),
                          expected_pixel_count);

  for (int i = 0; i < expected_pixel_count.size(); ++i) {
    int semantic_class_pixel_count = 0;
    const auto& semantic_label_blobs = graphLandmarkPtr->getBlobs()[i];
    for (int j = 0; j < semantic_label_blobs.size(); ++j)
      semantic_class_pixel_count += semantic_label_blobs[j].num_pixels_;
    CHECK_EQ(expected_pixel_count[i], semantic_class_pixel_count)
      << "In image " << imageName << ", class instance " << i
      << " should have " << expected_pixel_count[i] << " pixels, but has "
      << semantic_class_pixel_count;
  }
}

void testBlobsCount(const GraphLandmarkPtr& graphLandmarkPtr,
                    const std::vector<int>& expected_blob_count,
                    const std::string& imageName) {
  const auto& blobs = graphLandmarkPtr->getBlobs();
  for (int i = 0; i < expected_blob_count.size(); ++i) {
    CHECK_EQ(expected_blob_count[i], blobs[i].size())
      << "In image " << imageName << ", class " << i << " should have "
      << expected_blob_count[i] << " blobs, but has " << blobs[i].size();
  }
}

void createCustomImage(const int desiredRows, const int desiredCols,
                       cv::Mat& image) {

  image.create(desiredRows, desiredCols, CV_IMAGE_TYPE);
  for (int i = 0; i < desiredRows; ++i) {
    for (int j = 0; j < desiredCols; ++j) {
      if (i < desiredRows / 2) {
        if (j < desiredCols / 2) {
          image.at<cv::Vec3b>(i, j)[0] = static_cast<unsigned char>(0);
          image.at<cv::Vec3b>(i, j)[1] = static_cast<unsigned char>(50);
        } else {
          image.at<cv::Vec3b>(i, j)[0] = static_cast<unsigned char>(1);
          image.at<cv::Vec3b>(i, j)[1] = static_cast<unsigned char>(100);
        }
      } else {
        if (j < desiredCols / 2) {
          image.at<cv::Vec3b>(i, j)[0] = static_cast<unsigned char>(2);
          image.at<cv::Vec3b>(i, j)[1] = static_cast<unsigned char>(150);
        } else {
          image.at<cv::Vec3b>(i, j)[0] = static_cast<unsigned char>(3);
          image.at<cv::Vec3b>(i, j)[1] = static_cast<unsigned char>(200);
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

  for (int c = 0; c < centers.size(); ++c) {
    const int label = labels[c];
    const int instance_id = c + 1;
    cv::circle(image, centers[c], radii[c],
               cv::Scalar(label, instance_id, 0),
               -1, 8, 0);
  }

}

}