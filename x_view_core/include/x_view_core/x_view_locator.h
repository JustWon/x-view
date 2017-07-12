#ifndef X_VIEW_LOCATOR_H
#define X_VIEW_LOCATOR_H

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/parameters/parameters.h>

namespace x_view {

class Locator {
 public:
  static const std::unique_ptr<Parameters>& getParameters();
  static const std::unique_ptr<AbstractDataset>& getDataset();
  static void registerParameters(std::unique_ptr<Parameters> parameters);
  static void registerDataset(std::unique_ptr<AbstractDataset> dataset);

 private:
  static std::unique_ptr<Parameters> parameters_;
  static std::unique_ptr<AbstractDataset> dataset_;
};

}

#endif //X_VIEW_LOCATOR_H
