#ifndef X_VIEW_AIRSIM_DATASET_H
#define X_VIEW_AIRSIM_DATASET_H

#include <x_view_core/datasets/abstract_dataset.h>

#include <glog/logging.h>

#define AIRSIM_NUM_SEMANTIC_CLASSES 0

namespace x_view {

/**
 * \brief This class carries information about the Airsim dataset
 * \note Airsim dataset available at https://github.com/Microsoft/AirSim
 */
class AirsimDataset : public AbstractDataset {

 public:
  AirsimDataset() : AbstractDataset(AIRSIM_NUM_SEMANTIC_CLASSES) {
    CHECK(false) << "AirsimDataset not implemented yet";
  }

  virtual ~AirsimDataset(){}

  virtual const std::string datasetName() const {
    return std::string("Airsim Dataset");
  }

};

}

#undef AIRSIM_NUM_SEMANTIC_CLASSES

#endif //X_VIEW_AIRSIM_DATASET_H
