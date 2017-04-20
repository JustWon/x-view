#ifndef X_VIEW_SYNTHIA_DATASET_H
#define X_VIEW_SYNTHIA_DATASET_H

#include <x_view_core/datasets/abstract_dataset.h>

namespace x_view {

/**
 * \brief This class carries information about the Synthia dataset
 * \note Synthia dataset available at http://synthia-dataset.net/
 */
class SynthiaDataset : public AbstractDataset {

 public:
  SynthiaDataset();
  virtual ~SynthiaDataset(){}

  virtual const std::string datasetName() const {
    return std::string("Synthia Dataset");
  }

  virtual cv::Mat preprocessSemanticImage(const sensor_msgs::ImageConstPtr&
  msg) const;

 private:
  static const int SYNTHIA_NUM_SEMANTIC_CLASSES;

};

}

#endif //X_VIEW_SYNTHIA_DATASET_H
