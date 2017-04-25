#include <x_view_core/datasets/synthia_dataset.h>

#include <opencv2/core/core.hpp>

namespace x_view {

#define SYNTHIA_NUM_SEMANTIC_CLASSES 13

SynthiaDataset::SynthiaDataset()
    : AbstractDataset(SYNTHIA_NUM_SEMANTIC_CLASSES) {
  // see http://synthia-dataset.net/table-classes/ for class labels
  semantic_entities_ = {
      {"misc", 0},
      {"sky", 1},
      {"building", 2},
      {"road", 3},
      {"sidewalk", 4},
      {"fence", 5},
      {"vegetation", 6},
      {"pole", 7},
      {"car", 8},
      {"sign", 9},
      {"pedestrian", 10},
      {"cyclist", 11},
      {"lanemarking", 12}
  };

  CHECK(semantic_entities_.size() == SYNTHIA_NUM_SEMANTIC_CLASSES)
  << "Number of defined semantic entities differs from the one "
  << "specified in the header file:\n\tSYNTHIA_NUM_SEMANTIC_CLASSES = "
  << SYNTHIA_NUM_SEMANTIC_CLASSES << "\n\tdefined entities = "
  << semantic_entities_.size();
}

cv::Mat SynthiaDataset::convertSemanticImage(
    const sensor_msgs::ImageConstPtr& msg) const{

  const int msg_size = msg->data.size();
  const int step_size = msg->step;

  CHECK(!bool(msg->is_bigendian)) << "Message must be little endian";

  const int cols = msg->width;
  const int rows = msg->height;

  // Utility function to convert two consecutive bytes into a 16bits unsigned
  // int value. This function assumes little endiannes of the system
  auto toInt = [&](int index) -> unsigned char {
    unsigned char s = ((msg->data[index + 1] << 8) | msg->data[index]);
    return s;
  };

  // new image used as container for semantic labels and instances.
  // the first channel of this new image contains the semantic label
  // associated to each pixel
  // the second channel contains a unique ID associated to dynamic objects
  // the third channel is not used
  cv::Mat labelImage(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));

  // loop over the rows of the image implicitly stored into msg
  for (int i = 0; i < rows; ++i) {
    // loop over the cols of the image implicitly stored into msg
    for (int j = 0; j < cols; ++j) {
      // index of the pixel, need to have "6*j" because each pixel value is
      // stored into two consecutive bytes and there are three channels
      int idx = step_size * i + 6 * j;
      CHECK(idx < msg_size) << "Computed index is larger or equal to message "
          "size";

      // loop over the three channels and extract the semantic classes
      cv::Vec3b values;
      for (int c = 0; c < 3; ++c) {
        values[2 - c] = (uchar) std::max(0, std::min((uchar) (toInt(idx + 2 * c)),
                                                     (uchar) numSemanticClasses())-1);
      }
      labelImage.at<cv::Vec3b>(cv::Point(j, i)) = values;
    }

  }

  return labelImage;

}

#undef SYNTHIA_NUM_SEMANTIC_CLASSES

}
