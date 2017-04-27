#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/features/graph_descriptor.h>

namespace x_view {

GraphMatcher::GraphMatcher() {
}

GraphMatcher::~GraphMatcher() {
}

AbstractMatcher::MatchingResultPtr
GraphMatcher::match(const SemanticLandmarkPtr& queryLandmark) {

  // Extract and cast the descriptor associated to the queryLandmark
  auto graphDescriptor =
      std::dynamic_pointer_cast<const GraphDescriptor>
          (queryLandmark->getDescriptor());

  // Perform checks related to the cast
  CHECK(graphDescriptor != nullptr) << "Impossible to cast descriptor "
      "associated to queryLandmark to a 'const GraphDescriptor'";


  // create a matching result pointer which will be returned by this function
  auto matchingResult = std::make_shared<GraphMatchingResult>();

  // TODO: match the graph descriptor

  // return the matching result filled with the matches
  return matchingResult;

}

LandmarksMatcherPtr GraphMatcher::create() {
  return std::make_shared<GraphMatcher>();
}

}

