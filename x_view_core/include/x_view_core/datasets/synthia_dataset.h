#ifndef X_VIEW_SYNTHIA_DATASET_H
#define X_VIEW_SYNTHIA_DATASET_H

#include <x_view_core/datasets/abstract_dataset.h>

#include <glog/logging.h>

#define SYNTHIA_NUM_SEMANTIC_CLASSES 13

namespace x_view {

/**
 * \brief This class carries information about the Synthia dataset
 * \note Synthia dataset available at http://synthia-dataset.net/
 */
class SynthiaDataset : public AbstractDataset {

 public:
  SynthiaDataset() : AbstractDataset(SYNTHIA_NUM_SEMANTIC_CLASSES) {
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

    CHECK(semantic_entities_.size() == SYNTHIA_NUM_SEMANTIC_CLASSES) <<
     "Number of defined semantic entities differs from the one "
     "specified in the header file:\n\tSYNTHIA_NUM_SEMANTIC_CLASSES = " <<
     SYNTHIA_NUM_SEMANTIC_CLASSES << "\n\tdefined entities = " << semantic_entities_.size();
  }

  virtual ~SynthiaDataset(){}

  virtual const std::string datasetName() const {
    return std::string("Synthia Dataset");
  }

};

}

#undef SYNTHIA_NUM_SEMANTIC_CLASSES

#endif //X_VIEW_SYNTHIA_DATASET_H
