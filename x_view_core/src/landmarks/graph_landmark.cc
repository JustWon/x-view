#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>

namespace x_view {
GraphLandmark::GraphLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {

  // Graph descriptor filled up by this function
  Graph descriptor;
  auto& graph = descriptor.graph();

  // perform the image segmentation on the first channel of the image
  findBlobs();

  // create the descriptor stored in this landmark by generating a
  // VectorDescriptor containing the histogram data
  descriptor_ = std::make_shared<GraphDescriptor>(GraphDescriptor(descriptor));

}

void GraphLandmark::findBlobs() {

  const int dataset_size = globalDatasetPtr->numSemanticClasses();
  image_blobs_.resize(dataset_size);

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
        // current ID of pixel
        const int currentLabelId =
            static_cast<int>(label_image.at<uchar>(cv::Point(x, y)));
        cv::Rect rect;
        cv::Mat mask =
            cv::Mat::zeros(label_image.rows + 2, label_image.cols + 2, CV_8U);
        cv::floodFill(label_image, mask, cv::Point(x, y), 255,
                      &rect, 0, 0, 4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);

        // blob built around the pixel
        std::vector<cv::Point> blob;

        for (int i = rect.y; i < (rect.y + rect.height); ++i) {
          for (int j = rect.x; j < (rect.x + rect.width); ++j) {
            cv::Point pixel(j, i);
            if (static_cast<int>(label_image.at<uchar>(pixel))
                == currentLabelId) {
              blob.push_back(pixel);
              alreadyTaken.insert(pixel);
            } else {
            }
          }
        }
        image_blobs_[currentLabelId].push_back(blob);
      }
    }
  }
}

void GraphLandmark::printBlobs(std::ostream& out) const {
  for (int c = 0; c < image_blobs_.size(); ++c) {
    out << "Found " << image_blobs_[c].size()
        << " instances of class " << c << ":" << std::endl;
    for (int i = 0; i < image_blobs_[c].size(); ++i) {
      out << "\tInstance " << i << " composed by " << image_blobs_[c][i].size()
          << " pixels." << std::endl;
    }
    out << std::endl;
  }
}
}

