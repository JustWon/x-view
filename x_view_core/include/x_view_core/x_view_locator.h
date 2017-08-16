#ifndef X_VIEW_LOCATOR_H
#define X_VIEW_LOCATOR_H

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/parameters/parameters.h>
#include <x_view_core/timer/abstract_timer.h>

namespace x_view {

class Locator {
 public:
  static const std::unique_ptr<Parameters>& getParameters();
  static const std::unique_ptr<AbstractDataset>& getDataset();
  static const std::unique_ptr<AbstractTimer>& getTimer();

  static void registerParameters(std::unique_ptr<Parameters> parameters);
  static void registerDataset(std::unique_ptr<AbstractDataset> dataset);
  static void registerTimer(std::unique_ptr<AbstractTimer> timer);

  static AbstractTimer* removeTimer();

 private:
  static std::unique_ptr<Parameters> parameters_;
  static std::unique_ptr<AbstractDataset> dataset_;
  static std::unique_ptr<AbstractTimer> timer_;
};

}

#endif //X_VIEW_LOCATOR_H
