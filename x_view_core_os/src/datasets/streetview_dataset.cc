#include <x_view_core_os/datasets/streetview_dataset.h>
#include <x_view_core_os/x_view_tools.h>

namespace x_view {

#define STREETVIEW_NUM_SEMANTIC_CLASSES 13

StreetviewDataset::StreetviewDataset()
: AbstractDataset(STREETVIEW_NUM_SEMANTIC_CLASSES) {
  // See https://github.com/ethz-asl/x-view/tree/master/dataset_tools/streetview_to_rosbag
  // for the available class labels in our Streetview datasets.

  // Initialize the entities as:
  // {name, id, is_to_include_in_graph, is_static, is_to_render}
  semantic_entities_ = {
      {"sky", 0, false, true, false},
      {"building", 1, false, true, true},
      {"pole", 2, false, true, true},
      {"road marking", 3, false, true, true},
      {"road", 4, true, true, true},
      {"pavement", 5, true, true, true},
      {"tree", 6, true, true, true},
      {"signsymbol", 7, false, true, true},
      {"fence", 8, true, true, true},
      {"car", 9, true, true, true},
      {"pedestrian", 10, false, false, true},
      {"bicyclist", 11, true, false, true},
      {"unlabelled", 12, false, false, true}
  };

  CHECK(semantic_entities_.size() == STREETVIEW_NUM_SEMANTIC_CLASSES)
  << "Number of defined semantic entities differs from the one "
  << "specified in the header file:\n\tSTREETVIEW_NUM_SEMANTIC_CLASSES = "
  << STREETVIEW_NUM_SEMANTIC_CLASSES << "\n\tdefined entities = "
  << semantic_entities_.size();

  // Set the intrinsic parameters for the Streetview dataset.
  const real_t streetview_focal_length = 240;
  const int streetview_px = 240;
  const int streetview_py = 180;
  camera_intrinsics_ = CameraIntrinsics(streetview_focal_length,
                                        streetview_px, streetview_py);

  // Set up camera-to-image rotation.
  Matrix3r rotation;
  rotation <<
      0.0, -1.0,  0.0,
      0.0,  0.0, -1.0,
      1.0,  0.0,  0.0;


  camera_to_image_rotation_ = rotation;
}

cv::Mat StreetviewDataset::convertSemanticImage(
    const sensor_msgs::ImageConstPtr& msg) const {

  const int msg_size = static_cast<int>(msg->data.size());
  const int step_size = msg->step;

  CHECK(!bool(msg->is_bigendian))
  << "Message passed to StreetviewDataset must be little endian";

  const int cols = msg->width;
  const int rows = msg->height;

  // New image used as container for semantic labels and instances.
  // the first channel of this new image contains the semantic label
  // associated to each pixel
  // the second channel contains a unique ID associated to dynamic objects
  // the third channel is not used.
  cv::Mat labelImage(rows, cols, CV_8UC3, cv::Scalar::all(0));

  // Loop over the rows of the image implicitly stored into msg.
  for (int i = 0; i < rows; ++i) {
    // Loop over the cols of the image implicitly stored into msg.
    for (int j = 0; j < cols; ++j) {
      // Index of the pixel, need to have "3*j" because each pixel value is
      // stored into a single byte and there are three channels.
      int idx = step_size * i + 3 * j;
      CHECK(idx < msg_size)
      << "Computed index is larger or equal to message size";

      cv::Vec3b values;
      values[0] = msg->data[idx];
      values[1] = static_cast<uchar>(0);
      values[2] = static_cast<uchar>(0);

      labelImage.at<cv::Vec3b>(cv::Point2i(j, i)) = values;
    }

  }
  return labelImage;
}

const x_view::real_t StreetviewDataset::getDepth(const cv::Point2i& pixel,
                                             const cv::Mat& depth_image) const {
  const uint8_t point_depth = depth_image.at<uint8_t>(pixel);
  return point_depth * 100.0 / 256.0;
}

#undef STREETVIEW_NUM_SEMANTIC_CLASSES

}
