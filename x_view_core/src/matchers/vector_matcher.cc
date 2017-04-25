#include <x_view_core/matchers/vector_matcher.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/features/vector_descriptor.h>

namespace x_view {

VectorMatcher::VectorMatcher()
    : num_retained_best_matches_(1) {
  descriptor_matcher_ = std::make_shared<cv::BFMatcher>();
}

VectorMatcher::~VectorMatcher() {
}

AbstractMatcher::MatchingResultPtr
VectorMatcher::match(const SemanticLandmarkPtr& queryLandmark) {

  // Extract and cast the descriptor associated to the queryLandmark
  auto vectorDescriptor =
      std::dynamic_pointer_cast<const VectorDescriptor>(queryLandmark->getDescriptor());

  // Perform checks related to the cast
  CHECK(vectorDescriptor != nullptr) << "Impossible to cast descriptor "
      "associated to queryLandmark to a 'const VectorDescriptor'";


  // create a matching result pointer which will be returned by this function
  auto matchingResult = std::make_shared<VectorMatchingResult>();

  // match the vector descriptor to the descriptors previously visited
  // and store the results (matches) inside the matchingResult object
  descriptor_matcher_->knnMatch(vectorDescriptor->getDescriptor(),
                                matchingResult->matches,
                                num_retained_best_matches_);

  // add the descriptor of the queryLandmark to the set of features stored
  // inside the matcher
  descriptor_matcher_->add(std::vector<cv::Mat>{
      vectorDescriptor->getDescriptor()});

  // return the matching result filled with the matches
  return matchingResult;

}

LandmarksMatcherPtr VectorMatcher::create() {
  return std::make_shared<VectorMatcher>(VectorMatcher());
}

}

