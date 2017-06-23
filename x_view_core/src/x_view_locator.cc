#include <x_view_core/x_view_locator.h>

namespace x_view {

std::unique_ptr<Parameters> Locator::parameters_(new Parameters("EMPTY"));

std::unique_ptr<AbstractDataset> Locator::dataset_(new AbstractDataset(-1));

const std::unique_ptr<Parameters>& Locator::getParameters() {
  return Locator::parameters_;
}

const std::unique_ptr<AbstractDataset>& Locator::getDataset() {
  return Locator::dataset_;
}

void Locator::registerParameters(std::unique_ptr<Parameters> parameters) {

  parameters_ = std::move(parameters);
}

void Locator::registerDataset(std::unique_ptr<AbstractDataset> dataset) {
  dataset_ = std::move(dataset);
}

}

