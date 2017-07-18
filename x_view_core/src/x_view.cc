#include <x_view_core/x_view.h>

#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/landmarks/histogram_landmark.h>
#include <x_view_core/landmarks/visual_descriptor_landmark.h>
#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/matchers/vector_matcher.h>
#include <x_view_core/x_view_tools.h>

namespace x_view {

XView::XView()
    : frame_number_(-1) {
  printInfo();
  initialize();
}

void XView::processFrameData(const FrameData& frame_data) {

  ++frame_number_;
  LOG(INFO) << "XView starts processing frame " << frame_number_ << ".";
  const Eigen::RowVector3d origin = frame_data.getPose().getPosition();
  const Eigen::Matrix3d rotation = frame_data.getPose().getRotationMatrix();
  LOG(INFO) << "Associated robot pose:\n" << formatSE3(frame_data.getPose(),
                                                       "\t\t", 3);

  // Generate a new semantic landmark pointer.
  SemanticLandmarkPtr landmark_ptr;

  // Extract semantics associated to the semantic image and pose.
  createSemanticLandmark(frame_data, landmark_ptr);
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


}

const Graph& XView::getSemanticGraph() const {
  auto graph_matcher =
      std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_);

  CHECK_NOTNULL(graph_matcher.get());

  return graph_matcher->getGlobalGraph();
}

void XView::writeGraphToFile() const {
  const std::string filename = getOutputDirectory() + "merged_" +
      padded_int(frame_number_, 5, '0') + ".dot";
  const auto& graph_matcher =
      std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_);
  CHECK_NOTNULL(graph_matcher.get());
  writeToFile(graph_matcher->getGlobalGraph(), filename);
}

bool XView::localize(const FrameData& frame_data,
                     Eigen::Vector3d* position) {
  LOG(INFO) << "XView tries to localize a robot by its observations.";

  const cv::Mat& depth_image = frame_data.getDepthImage();

  // Generate a new semantic landmark pointer.
  SemanticLandmarkPtr landmark_ptr;

  // Extract semantics associated to the semantic image and pose.
  createSemanticLandmark(frame_data, landmark_ptr);

  const Graph& query_graph = std::dynamic_pointer_cast<const GraphDescriptor>(
  std::dynamic_pointer_cast<GraphLandmark>
      (landmark_ptr)->getDescriptor())->getDescriptor();

  // Get the existing global semantic graph before matching.
  const Graph& global_graph = getSemanticGraph();

  // Perform full matching getting similarity scores between query graph and
  // existing global semantic graph.
  GraphMatcher::GraphMatchingResult matching_result;

  // Extract the random walks of the graph.
  RandomWalkerParams random_walker_params_;
  random_walker_params_.num_walks = Locator::getParameters()
      ->getChildPropertyList("matcher")->getInteger("num_walks");
  random_walker_params_.walk_length = Locator::getParameters()
      ->getChildPropertyList("matcher")->getInteger("walk_length");
  RandomWalker random_walker(query_graph, random_walker_params_);
  random_walker.generateRandomWalks();

  // Create a matching result pointer which will be returned by this
  // function which stores the similarity matrix.

  GraphMatcher::SimilarityMatrixType& similarity_matrix =
      matching_result.getSimilarityMatrix();
  VectorXb& invalid_matches = matching_result.getInvalidMatches();

  std::dynamic_pointer_cast <GraphMatcher> (descriptor_matcher_)
              ->computeSimilarityMatrix(
                  random_walker, &similarity_matrix, &invalid_matches,
                  VertexSimilarity::SCORE_TYPE::WEIGHTED);

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_matrix =
      matching_result.computeMaxSimilarityRowwise().cwiseProduct(
          matching_result.computeMaxSimilarityColwise()
      );

  // Filter matches with geometric consistency.
  if (Locator::getParameters()->getChildPropertyList("matcher")->getBoolean(
      "outlier_rejection")) {
    bool filter_success = std::dynamic_pointer_cast < GraphMatcher
        > (descriptor_matcher_)->filter_matches(query_graph, global_graph,
                                                matching_result,
                                                &invalid_matches);
  }

  // Estimate transformation between graphs (Localization).
  GraphLocalizer graph_localizer;

  for(int j = 0; j < max_similarity_matrix.cols(); ++j) {
    int max_i = -1;
    max_similarity_matrix.col(j).maxCoeff(&max_i);
    if(max_i == -1)
      continue;
    if (!invalid_matches(j)) {
      std::cout << "Match between vertex " << j << " in query graph is vertex "
          <<max_i << " in global graph" << std::endl;
      const double similarity = similarity_matrix(max_i, j);
      const VertexProperty& match_v_p = global_graph[max_i];

      const unsigned short depth_cm =
          depth_image.at<unsigned short>(match_v_p.center);
      const double depth_m = depth_cm * 0.01;

      graph_localizer.addObservation(match_v_p, depth_m, similarity);
    }
  }

  SE3 transformation;
  bool localized = graph_localizer.localize(matching_result, query_graph, global_graph, &transformation);
  (*position) = transformation.getPosition();
  return localized;
}

void XView::printInfo() const {
  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");
  const auto& matcher_parameters =
      parameters->getChildPropertyList("matcher");
  const auto& localizer_parameters =
      parameters->getChildPropertyList("localizer");
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
      << "\n\tLocalizer type: \t<" + localizer_parameters->getString("type") + ">"
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
//         FUNCTIONS CALLED BY 'processFrameData' FUNCTION                  //
//==========================================================================//

void XView::createSemanticLandmark(const FrameData& frame_data,
                                   SemanticLandmarkPtr& semantics_out) const {

  // TODO: preprocess image and pose

  // Create the actual landmark representation whose implementation depends
  // on the parameters passed to XView.
  semantics_out = semantic_landmark_factory_.createSemanticLandmark(frame_data);

  LOG(INFO) << "XView created semantic landmark.";
}

void XView::matchSemantics(const SemanticLandmarkPtr& semantics_a,
                           AbstractMatcher::MatchingResultPtr& matching_result) {

  matching_result = descriptor_matcher_->match(semantics_a);

  const auto graph_matcher =
      std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_);

  const cv::Mat& current_semantic_image = semantics_a->getSemanticImage();
  const auto& current_graph_landmark =
      std::dynamic_pointer_cast<GraphLandmark>(semantics_a);
  CHECK_NOTNULL(current_graph_landmark.get());

  const auto& current_graph_descriptor =
      std::dynamic_pointer_cast<const GraphDescriptor>
          (current_graph_landmark->getDescriptor());

  const auto& current_graph =
      current_graph_descriptor->getDescriptor();

  const auto& graph_matching_result = std::dynamic_pointer_cast
      <GraphMatcher::GraphMatchingResult>(matching_result);

#ifdef X_VIEW_DEBUG
  cv::Mat current_image = GraphDrawer::createImageWithLabels
      (current_graph_landmark->getBlobs(), current_graph,
       semantics_a->getSemanticImage().size());

  cv::imshow("Semantic image ", current_image);
  cv::imshow("Similarity matrix ",
             SimilarityPlotter::getImageFromSimilarityMatrix(
                 graph_matching_result->getSimilarityMatrix()));
  cv::waitKey();
#endif

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
