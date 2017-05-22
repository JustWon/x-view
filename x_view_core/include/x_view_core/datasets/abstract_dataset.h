#ifndef X_VIEW_ABSTRACT_DATASET_H
#define X_VIEW_ABSTRACT_DATASET_H

#include <x_view_core/x_view_types.h>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <vector>
#include <string>

namespace x_view {

class AbstractDataset {
 public:

  /**
   * \brief A semantic entity is represented by a human readable name and by
   * an ID ranging from 0 to num_semantic_classes_-1.
   */
  struct SemanticEntity {
    SemanticEntity() {}
    SemanticEntity(const std::string& name, const int id,
                   const bool is_static = true,
                   const bool is_to_render = true)
        : semantic_entity_name_(name),
          semantic_entity_id_(id),
          is_static_(is_static),
          is_to_render_(is_to_render) {}

    std::string semantic_entity_name_;
    int semantic_entity_id_;
    bool is_static_;
    bool is_to_render_;

  };

  AbstractDataset(const int num_semantic_classes);
  virtual ~AbstractDataset() {}

  /**
   * \brief Dataset name.
   * \return Human readable name description of dataset.
   */
  virtual const std::string datasetName() const {
    return std::string("Abstract Dataset");
  };

  ///\brief Generates a human readable description of the database, t is the
  /// standard indentation to be used between new lines of the generated string.
  virtual const std::string datasetInfo(const std::string& t = "") const;

  ///\brief Returns the number of semantic classes contained in the dataset.
  int numSemanticClasses() const { return num_semantic_classes_; }

  ///\brief Returns the semantic entities associated to this dataset.
  const std::vector<SemanticEntity>& semanticEntities() const {
    return semantic_entities_;
  }

  ///\brief Returns the label (string) associated to a given index.
  const std::string& label(const int index) const {
    CHECK(index >= 0 && index < num_semantic_classes_);
    return semantic_entities_[index].semantic_entity_name_;
  }

  /**
   * \brief Function called by ROS each time a new semantic image is available.
   * \details This function is called by the x_view worker before passing the
   * image to the x_view.
   */
  virtual cv::Mat convertSemanticImage(const sensor_msgs::ImageConstPtr&
  msg) const;

  /// \brief Since some class labels are too general (e.g. SYNTHIA::MISC),
  /// this function returns a vector of labels that are not, thus labels that
  /// one might want to render.
  virtual const std::vector<int> getLabelsToRender() const;

  /// \brief returns a vector containing the index of the semantic entities
  /// being static.
  virtual const std::vector<int> getStaticLabels() const;

  /// \brief returns a vector containing the index of the semantic entities
  /// being dynamic.
  virtual const std::vector<int> getDynamicLabels() const;

 protected:
  const int num_semantic_classes_;
  std::vector<SemanticEntity> semantic_entities_;
};

/// \brief Dataset accessible from everywhere in the x_view project.
extern ConstDatasetPrt global_dataset_ptr;

}

#endif //X_VIEW_ABSTRACT_DATASET_H
