#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include <x_view_core/datasets/abstract_dataset.h>

namespace enc = sensor_msgs::image_encodings;

namespace x_view {

AbstractDataset::AbstractDataset(const int num_semantic_classes)
    : num_semantic_classes_(num_semantic_classes) {
  // create simple semantic entities
  for(int i = 0; i< num_semantic_classes_; ++i) {
    // Default semantic entities, all static and all to render
    semantic_entities_.push_back(SemanticEntity(std::to_string(i), i));
  }
}

const std::string AbstractDataset::datasetInfo(const std::string& t) const {
  std::string description = t + datasetName() + ":\n";
  for (auto elem : semantic_entities_) {
    description += t + "\t" + std::to_string(elem.semantic_entity_id_) + ": ";
    description += elem.semantic_entity_name_;
    description += "\n";
  }
  return description;
}

cv::Mat AbstractDataset::convertSemanticImage(
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

const std::vector<int> AbstractDataset::getLabelsToRender() const {
  std::vector<int> labels_to_render;
  labels_to_render.reserve(num_semantic_classes_);
  for(auto const& c : semantic_entities_)
    if(c.is_to_render_)
      labels_to_render.push_back(c.semantic_entity_id_);

  return labels_to_render;
}

}
