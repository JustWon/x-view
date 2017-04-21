#include <x_view_core/datasets/abstract_dataset.h>

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>

namespace enc = sensor_msgs::image_encodings;

namespace x_view {

AbstractDataset::AbstractDataset(const int num_semantic_classes)
    : num_semantic_classes_(num_semantic_classes) {}

const std::string AbstractDataset::datasetInfo(const std::string& t) const {
  std::string description = t + datasetName() + ":\n";
  for (auto elem : semantic_entities_) {
    description += t + "\t" + std::to_string(elem.semantic_entity_id_) + ": ";
    description += elem.semantic_entity_name_;
    description += "\n";
  }
  return description;
}

cv::Mat AbstractDataset::preprocessSemanticImage(
    const sensor_msgs::ImageConstPtr& msg) const {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

    cv::Mat image = cv_ptr->image;

    return image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("Could not convert from '"
                         << msg->encoding
                         << "' to '" << enc::BGR8 << "'"
                         << "\nError: " << e.what());
  }
}

}
