#include <x_view_core/matchers/vector_matcher.h>

#include <x_view_core/features/vector_descriptor.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/x_view_locator.h>

namespace x_view {

VectorMatcher::VectorMatcher() {
  const auto& parameters = Locator::getParameters();
  const auto& matcher_parameters = parameters->getChildPropertyList("matcher");
  num_retained_best_matches_ =
      matcher_parameters->getInteger("num_retained_matches", 1);

  descriptor_matcher_ = std::make_shared<cv::BFMatcher>();
}

VectorMatcher::~VectorMatcher() {
}

AbstractMatcher::MatchingResultPtr
VectorMatcher::match(const SemanticLandmarkPtr& query_landmark) {

  // Extract and cast the descriptor associated to the queryLandmark
  auto vector_descriptor =
      std::dynamic_pointer_cast<const VectorDescriptor>(query_landmark->getDescriptor());

  // Perform checks related to the cast
  CHECK_NOTNULL(vector_descriptor.get());

  // create a matching result pointer which will be returned by this function
  auto matching_result = std::make_shared<VectorMatchingResult>();

  // match the vector descriptor to the descriptors previously visited
  // and store the results (matches) inside the matchingResult object
  descriptor_matcher_->knnMatch(vector_descriptor->getDescriptor(),
                                matching_result->matches,
                                num_retained_best_matches_);

  // add the descriptor of the queryLandmark to the set of features stored
  // inside the matcher
  descriptor_matcher_->add(std::vector<cv::Mat>{
      vector_descriptor->getDescriptor()});

  // return the matching result filled with the matches
  return matching_result;

}

bool VectorMatcher::estimateTransformation(
    const MatchingResultPtr matching_result, SE3* transformation) {
  return false;
}

LandmarksMatcherPtr VectorMatcher::create() {
  return std::make_shared<VectorMatcher>();
}

}

