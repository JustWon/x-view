#include <x_view_core/bos_trainer.h>
#include <x_view_core/visual_feature.h>

namespace x_view {

BoSTrainer::BoSTrainer(int clusterCount,
                       const cv::TermCriteria& termcrit,
                       int attempts,
                       int flags)
    : cv::BOWKMeansTrainer(clusterCount, termcrit, attempts, flags) {

}

BoSTrainer::~BoSTrainer() {

}

void BoSTrainer::add(const SemanticLandmarkPtr& semanticLandmarkPtr) {
  std::shared_ptr<BoS> bos = std::dynamic_pointer_cast<BoS>(semanticLandmarkPtr);
  CHECK(bos != nullptr) << "semanticLandmarkPtr passed to " << __FUNCTION__
                        << "might not be an instance of 'BoS' class";
}

}

