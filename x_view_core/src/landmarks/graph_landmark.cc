#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>

namespace x_view {
GraphLandmark::GraphLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {

  std::cout << "Called GraphLandmark constructor " << std::endl;

  // Graph descriptor filled up by this function
  Graph descriptor;
  auto& graph = descriptor.graph();

  std::cout << "Here" << std::endl;

  // perform the image segmentation on the first channel of the image

  // outer dimension: 'blobs.size()' = dataset size
  // second dimension: 'blobs[i].size()' = number of disconnected instances of
  // class 'i'
  // third dimension: 'blobs[i][j].size()' = number of pixels of 'j'-th
  // instance of class 'i'
  std::vector<std::vector<std::vector<cv::Point>>> blobs;
  std::cout << "Ready to call blobs" << std::endl;
  findBlobs(blobs);
  std::cout << "Blobs computed" << std::endl;

  for (int c = 0; c < blobs.size(); ++c) {
    std::cout << "Class " << globalDatasetPtr->label(c) << " has "
              << blobs[c].size() << " instances " << std::endl;
    for (int i = 0; i < blobs[c].size(); ++i) {
      std::cout << "Instance " << i << " has " << blobs[c][i].size()
                << " pixels" << std::endl;
    }
  }


  // create the descriptor stored in this landmark by generating a
  // VectorDescriptor containing the histogram data
  descriptor_ = std::make_shared<GraphDescriptor>(GraphDescriptor(descriptor));

}

void GraphLandmark::findBlobs(
    std::vector<std::vector<std::vector<cv::Point>>>& blobs) const {

  const int dataset_size = globalDatasetPtr->numSemanticClasses();
  blobs.resize(dataset_size);

  // "channels" is a vector of 3 Mat arrays:
  std::vector<cv::Mat> channels(3);
  cv::split(semantic_image_, channels);
  const cv::Mat label_image = channels[0];

  auto PointComp = [](const cv::Point& p1, const cv::Point& p2) -> bool {
    return std::tie(p1.x, p1.y) < std::tie(p2.x, p2.y);
  };

  std::set<cv::Point, decltype(PointComp)> alreadyTaken(PointComp);

  for (int y = 0; y < label_image.rows; y++) {
    for (int x = 0; x < label_image.cols; x++) {
      if (alreadyTaken.find(cv::Point(x, y)) == alreadyTaken.end()) {
        std::cout << "Traversing pixel " << cv::Point(x, y) << std::endl;
        // current ID of pixel
        const int currentLabelId = label_image.at<int>(cv::Point(x, y));
        cv::Rect rect;
        cv::Mat localImage = label_image;
        cv::floodFill(localImage, cv::Point(x, y), currentLabelId,
                      &rect, 0, 0, 4);
        
        // blob built around the pixel
        std::vector<cv::Point> blob;

        std::cout << "Rect: " << rect.x << ", " << rect.y
                  << "; " << rect.width << ", "
                      "" << rect.height << std::endl;

        for (int i = rect.y; i < (rect.y + rect.height); i++) {
          int* row2 = (int*) label_image.ptr(i);
          for (int j = rect.x; j < (rect.x + rect.width); j++) {
            if (row2[j] == currentLabelId) {
              cv::Point pixel(j, i);
              blob.push_back(pixel);
              alreadyTaken.insert(pixel);
            }
          }
        }
        std::cout << "Found a blob of size " << blob.size() << std::endl;
        blobs[currentLabelId].push_back(blob);
      }
    }
  }
}
}

