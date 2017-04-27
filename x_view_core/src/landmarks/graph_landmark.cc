#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>

#include <opencv2/imgproc/imgproc.hpp>

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

  // this container contains all pixels that have already been inserted into a
  // blob, it is used to avoid counting the same pixel multiple times
  std::set<cv::Point, decltype(PointComp)> pixels_in_blob(PointComp);

  // variable used for the flood algorithm
  cv::Mat flood_mask =
      cv::Mat::zeros(label_image.rows + 2, label_image.cols + 2, CV_8U);

  // helper function to get the label associated to a pixel
  auto getPixelLabel = [&](const cv::Point& pixel) -> int {
    return static_cast<int>(label_image.at<uchar>(pixel));
  };

  for (int y = 0; y < label_image.rows; y++) {
    for (int x = 0; x < label_image.cols; x++) {
      if (pixels_in_blob.find(cv::Point(x, y)) == pixels_in_blob.end()) {
        const cv::Point seed_pixel(x, y);

        const int seed_pixel_label = getPixelLabel(seed_pixel);

        cv::Rect rect;
        cv::floodFill(label_image, flood_mask, seed_pixel, 255,
                      &rect, 0, 0, 4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);

        // build a blob around the seed pixel
        std::vector<cv::Point> blob_around_seed_pixel;

        for (int i = rect.y; i < (rect.y + rect.height); ++i) {
          for (int j = rect.x; j < (rect.x + rect.width); ++j) {
            const cv::Point query_pixel(j, i);
            if (getPixelLabel(query_pixel) == seed_pixel_label) {
              bool has_query_pixel_been_inserted_now =
                  pixels_in_blob.insert(query_pixel).second;
              if (has_query_pixel_been_inserted_now)
                blob_around_seed_pixel.push_back(query_pixel);
            } else {
            }
          }
        }
        image_blobs_[seed_pixel_label].push_back(blob_around_seed_pixel);
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

const cv::Mat GraphLandmark::getImageFromBlobs() const {
  cv::Mat resImage(semantic_image_.rows, semantic_image_.cols,
                   CV_8UC1, cv::Scalar::all(0));
  for (int l = 0; l < image_blobs_.size(); ++l) {
    for (int i = 0; i < image_blobs_[l].size(); ++i) {
      for (auto p : image_blobs_[l][i])
        resImage.at<uchar>(p) =
            static_cast<uchar>( l *
                (255. / (globalDatasetPtr->numSemanticClasses() - 1)));
    }
  }
  return resImage;
}
}

