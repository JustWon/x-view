#include <x_view_core/datasets/airsim_dataset.h>

namespace x_view {

#define AIRSIM_NUM_SEMANTIC_CLASSES 0

AirsimDataset::AirsimDataset()
    : AbstractDataset(AIRSIM_NUM_SEMANTIC_CLASSES) {

  CHECK(false) << "AirsimDataset not implemented yet";

  semantic_entities_ = {  };

  CHECK(semantic_entities_.size() == AIRSIM_NUM_SEMANTIC_CLASSES)
  << "Number of defined semantic entities differs from the one "
  << "specified in the header file:\n\tAIRSIM_NUM_SEMANTIC_CLASSES = "
  << AIRSIM_NUM_SEMANTIC_CLASSES << "\n\tdefined entities = "
  << semantic_entities_.size();
}

#undef AIRSIM_NUM_SEMANTIC_CLASSES

}

