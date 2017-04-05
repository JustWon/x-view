#ifndef X_VIEW_ABSTRACT_DATASET_H
#define X_VIEW_ABSTRACT_DATASET_H

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
    SemanticEntity(const std::string& name, const int id) :
        semantic_entity_name_(name), semantic_entity_id_(id) {}

    std::string semantic_entity_name_;
    int semantic_entity_id_;
  };

  AbstractDataset(const int num_semantic_classes)
      : num_semantic_classes_(num_semantic_classes) {}

  virtual ~AbstractDataset() {}

  /**
   * \brief Pure virtual function used to avoid an instantiation of this class
   * \return Human readable name description of dataset
   */
  virtual const std::string& datasetName() const = 0;

  ///\brief returns the number of semantic classes contained in the dataset
  int numSemanticClasses() const { return num_semantic_classes_; }

  ///\brief returns the semantic entities associated to this dataset
  const std::vector <std::string>& semanticEntities() const {
    return semantic_entities_;
  }

 private:
  const int num_semantic_classes_;

 protected:
  std::vector <SemanticEntity> semantic_entities_;
};

}

#endif //X_VIEW_ABSTRACT_DATASET_H
