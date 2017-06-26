#include <x_view_core/x_view.h>

#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/landmarks/histogram_landmark.h>
#include <x_view_core/landmarks/visual_descriptor_landmark.h>
#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/matchers/vector_matcher.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

namespace x_view {

XView::XView()
    : frame_number_(0) {
  printInfo();
  initialize();
}

void XView::processSemanticImage(const cv::Mat& image, const SE3& pose) {

  LOG(INFO) << "XView starts processing frame " << frame_number_ << ".";

  // Generate a new semantic landmark pointer.
  SemanticLandmarkPtr landmark_ptr;

  // Extract semantics associated to the semantic image and pose.
  extractSemanticsFromImage(image, pose, landmark_ptr);

  // Compute the matches between the new feature and the ones
  // stored in the database.
  if (frame_number_ == 0) {
    // Simply add the landmark to the matcher without matching anything.
    descriptor_matcher_->addDescriptor(landmark_ptr->getDescriptor());
  } else {
    // Perform full matching.
    AbstractMatcher::MatchingResultPtr matching_result_ptr;
    matchSemantics(landmark_ptr, matching_result_ptr);
  }

  // Add the semantic landmark to the database.
  semantics_db_.push_back(landmark_ptr);

  LOG(INFO) << "XView ended processing frame " << frame_number_ << ".";
  ++frame_number_;
}

void XView::printInfo() const {
  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");
  const auto& matcher_parameters =
      parameters->getChildPropertyList("matcher");
  const auto& dataset = Locator::getDataset();

  LOG(INFO)
      << "\n==========================================================\n"
      << "                  XView"
      #ifdef X_VIEW_DEBUG
      << " (Debug)"
      #else
      << " (Release)"
      #endif
      << "\n\n" << dataset
      << "\n\tLandmark type:\t<" + landmark_parameters->getString("type") + ">"
      << "\n\tMatcher type: \t<" + matcher_parameters->getString("type") + ">"
      << "\n==========================================================\n";

  LOG(INFO)
      << "\nParameter tree:\n" << parameters->toString();
}

void XView::initialize() {
  initializeLandmarkFactory();
  initializeMatcher();
}

void XView::initializeLandmarkFactory() {
  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");
  const std::string landmark_type = landmark_parameters->getString("type");

  if (landmark_type == "ORB") {
    semantic_landmark_factory_.setCreatorFunction
        (ORBVisualDescriptorLandmark::create);
  } else if (landmark_type == "SIFT") {
    semantic_landmark_factory_.setCreatorFunction
        (SIFTVisualDescriptorLandmark::create);
  } else if (landmark_type == "SURF") {
    semantic_landmark_factory_.setCreatorFunction
        (SURFVisualDescriptorLandmark::create);
  } else if (landmark_type == "HISTOGRAM") {
    semantic_landmark_factory_.setCreatorFunction(HistogramLandmark::create);
  } else if (landmark_type == "GRAPH") {
    semantic_landmark_factory_.setCreatorFunction(GraphLandmark::create);
  } else {
    CHECK(false) << "Unrecognized landmark type <" << landmark_type << ">"
                 << std::endl;
  }
}

void XView::initializeMatcher() {
  const auto& parameters = Locator::getParameters();
  const auto& matcher_parameters =
      parameters->getChildPropertyList("matcher");
  const std::string matcher_type = matcher_parameters->getString("type");

  if (matcher_type == "VECTOR") {
    descriptor_matcher_ = VectorMatcher::create();
  } else if (matcher_type == "GRAPH") {
    descriptor_matcher_ = GraphMatcher::create();
  } else {
    CHECK(false) << "Unrecognized matcher type <" << matcher_type << ">"
                 << std::endl;
  }

}

//==========================================================================//
//         FUNCTIONS CALLED BY 'processSemanticImage' FUNCTION              //
//==========================================================================//

void XView::extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                      SemanticLandmarkPtr& semantics_out) {

  // TODO: preprocess image and pose

  // Create the actual landmark representation whose implementation depends
  // on the parameters passed to XView.
  semantics_out =
      semantic_landmark_factory_.createSemanticLandmark(image, pose);

  LOG(INFO) << "XView created semantic landmark.";
}

void XView::matchSemantics(const SemanticLandmarkPtr& semantics_a,
                           AbstractMatcher::MatchingResultPtr& matching_result) {

  const auto& old_graph_matcher = std::dynamic_pointer_cast<GraphMatcher>
      (descriptor_matcher_);

  // Peform the merging operation.
  std::shared_ptr<GraphMatcher> new_graph_matcher(new GraphMatcher);
  new_graph_matcher->addDescriptor(old_graph_matcher->getGlobalGraph());


  const cv::Mat& current_semantic_image = semantics_a->getSemanticImage();
  const auto& current_graph_landmark =
      std::dynamic_pointer_cast<GraphLandmark>(semantics_a);
  CHECK_NOTNULL(current_graph_landmark.get());

  const auto& current_graph_descriptor =
      std::dynamic_pointer_cast<const GraphDescriptor>
          (current_graph_landmark->getDescriptor());

  const Graph& current_graph = current_graph_descriptor->getDescriptor();

  // Compute a match between the current semantic landmark and the ones
  // already visited.
  matching_result =  new_graph_matcher->match(semantics_a);


  const auto& graph_matching_result = std::dynamic_pointer_cast
      <GraphMatcher::GraphMatchingResult>(matching_result);

  GraphMerger graph_merger(new_graph_matcher->getGlobalGraph(),
                           current_graph,
                           *(graph_matching_result.get()));
  Graph merged_graph = graph_merger.getMergedGraph();
  std::string filename = getOutputDirectory() + "merged_" + padded_int
      (frame_number_, 5, '0') + ".dot";
  dumpToDotFile(merged_graph, filename);

  descriptor_matcher_ = new_graph_matcher;

  std::cout << "Matched completed. Current global graph is being damped to "
      "file" + filename << std::endl;

  cv::Mat current_image = GraphDrawer::createImageWithLabels
      (current_graph_landmark->getBlobs(), current_graph,
       semantics_a->getSemanticImage().size());
  cv::imshow("Semantic image " + padded_int(frame_number_, 2, ' '), current_image);
  cv::waitKey();

}

void XView::filterMatches(const SemanticLandmarkPtr& semantics_a,
                          AbstractMatcher::MatchingResultPtr& matching_result) {
  // TODO: filter matches, e.g., with geometric verification.
  CHECK(false) << "Not implemented.";
}

void XView::mergeSemantics(const SemanticLandmarkPtr& semantics_a,
                           AbstractMatcher::MatchingResultPtr& matching_result) {
  // TODO: Merge semantics with semantics_db_ if dominant matches,
  // otherwise add semantics as new instance to semantics_db_.
  // TODO: use filterMatches function before merging.
  CHECK(false) << "Not implemented.";
}

void XView::cleanDatabase() {
  // TODO: sweep over semantics_db_ to match and merge unconnected semantics.
  CHECK(false) << "Not implemented.";
}

}
