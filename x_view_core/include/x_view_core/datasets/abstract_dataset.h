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
                   const bool is_to_include_in_graph = true,
                   const bool is_static = true,
                   const bool is_to_render = true)
        : semantic_entity_name_(name),
          semantic_entity_id_(id),
          is_to_include_in_graph_(is_to_include_in_graph),
          is_static_(is_static),
          is_to_render_(is_to_render) {}

    /// \brief Name associated to the semantic entity.
    std::string semantic_entity_name_;
    /// \brief Integer key (label) associated to the semantic entity.
    int semantic_entity_id_;
    /// \brief Flag indicating if this entity has to be included in the
    /// semantic graph construction or not.
    bool is_to_include_in_graph_;
    /// \brief Flag indicating if this entity is of static type or not.
    bool is_static_;
    /// \brief Flag indicating if blobs associated with this entity are to be
    /// rendered when displaying semantic images or not.
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

  /// \brief Returns a vector containing the index of the semantic entities
  /// being static.
  virtual const std::vector<int> getStaticLabels() const;

  /// \brief Returns a vector containing the index of the semantic entities
  /// being dynamic.
  virtual const std::vector<int> getDynamicLabels() const;

  /// \brief Returns a vector containing the index of the semantic entities
  /// to be included in the semantic graph construction. Unimportant or
  /// generic labels are excluded from the semantic graph, as they don't add
  /// any information to the scene.
  virtual const std::vector<int> getLabelsToIncludeInGraph() const;

  /// \brief Returns the length of the longest label in the dataset. This
  /// function is used for formatting the output.
  const unsigned long largestLabelSize() const;

 protected:
  const int num_semantic_classes_;
  std::vector<SemanticEntity> semantic_entities_;
};

/**
 * \brief Streams the dataset in a human readable way to the passed
 * stream argument.
 * \param out Stream object to be streamed to.
 * \param dataset AbstractDataset object to be streamed.
 * \return Stream filled with AbstaractDataset.
 */
std::ostream& operator<<(std::ostream& out, const AbstractDataset& dataset);

/// \brief Overloaded operator to print object pointed by ConstDatasetPtr.
std::ostream& operator<<(std::ostream& out, const ConstDatasetPtr& ptr);

/// \brief Dataset accessible from everywhere in the x_view project.
extern ConstDatasetPtr global_dataset_ptr;

}

#endif //X_VIEW_ABSTRACT_DATASET_H
