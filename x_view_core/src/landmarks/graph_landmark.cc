#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/features/graph_descriptor.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <bitset>

namespace x_view {

// FIXME: should this parameter be read by the config file?
int GraphLandmark::MINIMUM_BLOB_SIZE = 40;

GraphLandmark::GraphLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {

  // Graph descriptor filled up by this function
  Graph descriptor;
  auto& graph = descriptor.graph();

  // perform image segmentation on the first channel of the image
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
  cv::Mat label_image = channels[0];

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
        // we are only interested in large blobs!
        if (blob_around_seed_pixel.size() > MINIMUM_BLOB_SIZE)
          image_blobs_[seed_pixel_label].push_back(
              Blob(seed_pixel_label, blob_around_seed_pixel));
      }
    }
  }
}

void GraphLandmark::printBlobs(std::ostream& out) const {
  for (int c = 0; c < image_blobs_.size(); ++c) {
    out << "Found " << image_blobs_[c].size()
        << " instances of class " << c << ":" << std::endl;
    for (int i = 0; i < image_blobs_[c].size(); ++i) {
      out << "\tInstance " << i << " composed by "
          << image_blobs_[c][i].pixels_.size() << " pixels with mean pixel "
          << image_blobs_[c][i].center_ << std::endl;
    }
    out << std::endl;
  }
}

const cv::Mat GraphLandmark::getImageFromBlobs(
    const std::vector<int>& labels_to_render) const {
  cv::Mat resImage(semantic_image_.rows, semantic_image_.cols,
                   CV_8UC3, cv::Scalar::all(0));

  // Draw the blobs onto the image
  for (int l = 0; l < image_blobs_.size(); ++l) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), l) !=
        std::end(labels_to_render))
      for (int i = 0; i < image_blobs_[l].size(); ++i) {
        for (auto p : image_blobs_[l][i].pixels_) {
          const int semantic_label = l;
          const uchar intensity =
              static_cast<uchar>(255 / (semantic_label / 8 + 1));

          std::bitset<3> bits(semantic_label);
          resImage.at<cv::Vec3b>(p)[0] = bits[0] ? intensity : (uchar) 0;
          resImage.at<cv::Vec3b>(p)[1] = bits[1] ? intensity : (uchar) 0;
          resImage.at<cv::Vec3b>(p)[2] = bits[2] ? intensity : (uchar) 0;
        }

      }
  }

  // Draw the labels and the blobs centers
  const int circle_radius = 5;
  const int line_thickness = -1;
  const cv::Scalar center_color = cv::Scalar(255, 130, 100);
  const cv::Scalar font_color = cv::Scalar(255, 255, 255);
  for (int l = 0; l < image_blobs_.size(); ++l) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), l) !=
        std::end(labels_to_render))
      for (auto const& b : image_blobs_[l]) {
        cv::circle(resImage, b.center_, circle_radius,
                   center_color, line_thickness);
        cv::putText(resImage, globalDatasetPtr->label(b.semantic_label_),
                    b.center_, cv::FONT_HERSHEY_DUPLEX, 0.85, font_color,
                    1, CV_AA);
      }
  }

  return resImage;
}

}

