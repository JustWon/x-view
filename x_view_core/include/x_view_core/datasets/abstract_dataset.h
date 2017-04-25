#ifndef X_VIEW_ABSTRACT_DATASET_H
#define X_VIEW_ABSTRACT_DATASET_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

#include <sensor_msgs/Image.h>
#include <ros/ros.h>

#include <glog/logging.h>

#include <vector>
#include <string>

namespace x_view {

class AbstractDataset {
 public:

  /**
   * \brief A semantic entity is represented by a human readable name and by
   * an ID ranging from 0 to num_semantic_classes_-1
   */
  struct SemanticEntity {
    SemanticEntity() : semantic_entity_name_(""), semantic_entity_id_(-1) {}
    SemanticEntity(const std::string& name, const int id) :
        semantic_entity_name_(name), semantic_entity_id_(id) {}

    std::string semantic_entity_name_;
    int semantic_entity_id_;
  };

  AbstractDataset(const int num_semantic_classes);
  virtual ~AbstractDataset() {}

  /**
   * \brief Pure virtual function used to avoid an instantiation of this class
   * \return Human readable name description of dataset
   */
  virtual const std::string datasetName() const = 0;

  ///\brief Generates a human readable descripion of the database, t is the
  /// standard indentation to be used between new lines of the generated string
  virtual const std::string datasetInfo(const std::string& t = "") const;

  ///\brief returns the number of semantic classes contained in the dataset
  int numSemanticClasses() const { return num_semantic_classes_; }

  ///\brief returns the semantic entities associated to this dataset
  const std::vector<SemanticEntity>& semanticEntities() const {
    return semantic_entities_;
  }

  ///\brief returns the label (string) associated to a given index
  const std::string& label(const int index) const {
    CHECK(index >= 0 && index < num_semantic_classes_);
    return semantic_entities_[index].semantic_entity_name_;
  }

  /**
   * \brief Function called by ROS each time a new semantic image is available
   * \details This function is called by the x_view worker before passing the
   * image to the x_view
   */
  virtual cv::Mat convertSemanticImage(const sensor_msgs::ImageConstPtr&
  msg) const;

 protected:
  const int num_semantic_classes_;
  std::vector<SemanticEntity> semantic_entities_;
};


/// \brief dataset accessible from everywhere in the x_view project
extern ConstDatasetPrt globalDatasetPtr;

}

#endif //X_VIEW_ABSTRACT_DATASET_H
