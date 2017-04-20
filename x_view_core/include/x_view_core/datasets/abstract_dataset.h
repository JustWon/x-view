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
    SemanticEntity() : semantic_entity_name_(""), semantic_entity_id_(-1) {}
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
  virtual const std::string datasetName() const = 0;

  ///\brief Generates a human readable descripion of the database
  virtual const std::string datasetInfo() const {
    std::string description = datasetName() + ":\n";
    for(auto elem : semantic_entities_) {
      description += "\t";
      description += std::to_string(elem.semantic_entity_id_) + ": ";
      description += elem.semantic_entity_name_;
      description += "\n";
    }
    return description;
  }

  ///\brief returns the number of semantic classes contained in the dataset
  int numSemanticClasses() const { return num_semantic_classes_; }

  ///\brief returns the semantic entities associated to this dataset
  const std::vector<SemanticEntity>& semanticEntities() const {
    return semantic_entities_;
  }

  const std::string& label(const int index) const {
    CHECK(index >= 0 && index < num_semantic_classes_);
    return semantic_entities_[index].semantic_entity_name_;
  }

 private:
  const int num_semantic_classes_;

 protected:
  std::vector<SemanticEntity> semantic_entities_;
};

}

#endif //X_VIEW_ABSTRACT_DATASET_H
