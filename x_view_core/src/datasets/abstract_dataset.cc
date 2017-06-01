#include <x_view_core/datasets/abstract_dataset.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <iomanip>

namespace enc = sensor_msgs::image_encodings;

namespace x_view {

AbstractDataset::AbstractDataset(const int num_semantic_classes)
    : num_semantic_classes_(num_semantic_classes) {
  // create simple semantic entities
  for (int i = 0; i < num_semantic_classes_; ++i) {
    // Default semantic entities, all static and all to render
    semantic_entities_.push_back(SemanticEntity(std::to_string(i), i));
  }
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
  for (auto const& c : semantic_entities_)
    if (c.is_to_render_)
      labels_to_render.push_back(c.semantic_entity_id_);

  return labels_to_render;
}

const std::vector<int> AbstractDataset::getStaticLabels() const {
  std::vector<int> static_labels;
  static_labels.reserve(num_semantic_classes_);
  for (auto const& c : semantic_entities_)
    if (c.is_static_)
      static_labels.push_back(c.semantic_entity_id_);

  return static_labels;
}

const std::vector<int> AbstractDataset::getDynamicLabels() const {
  std::vector<int> dynamic_labels;
  dynamic_labels.reserve(num_semantic_classes_);
  for (auto const& c : semantic_entities_)
    if (!c.is_static_)
      dynamic_labels.push_back(c.semantic_entity_id_);

  return dynamic_labels;
}

const unsigned long AbstractDataset::largestLabelSize() const {
  return std::max_element(semantic_entities_.begin(), semantic_entities_.end(),
                          [](const SemanticEntity& s1,
                             const SemanticEntity& s2) {
                            return s1.semantic_entity_name_.length() <
                                s2.semantic_entity_name_.length();
                          })->semantic_entity_name_.length();
}

std::ostream& operator<<(std::ostream& out, const AbstractDataset& dataset) {
  out << "Dataset name: " << dataset.datasetName() << std::endl << std::endl;
  const unsigned long max_label_length = dataset.largestLabelSize()+1;
  out << std::setfill(' ');
  out << std::left << std::setw(4) << "id:"
      << std::left << std::setw(max_label_length) << "label:"
      << std::left << std::setw(17) << "static/dynamic"
      << std::left << std::setw(10) << "drawable";
  for (const auto& elem : dataset.semanticEntities()) {
    out << "\n" << std::left << std::setw(4) << elem.semantic_entity_id_;
    out << std::left << std::setw(max_label_length) << elem.semantic_entity_name_;
    out << std::left << std::setw(17) << (elem.is_static_ ? "static"
                                                          : "dynamic");
    out << std::left << std::setw(10) << (elem.is_to_render_ ? "yes" : "no");
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const ConstDatasetPtr& ptr) {
  return out << *ptr;
}

}
