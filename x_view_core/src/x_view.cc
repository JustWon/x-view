#include <x_view_core/x_view.h>

#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/landmarks/histogram_landmark.h>
#include <x_view_core/landmarks/visual_descriptor_landmark.h>
#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/matchers/vector_matcher.h>
#include <x_view_core/x_view_tools.h>
#include <x_view_core/x_view_types.h>

#include <boost/graph/random.hpp>

#include <random>

namespace x_view {

XView::XView()
    : frame_number_(-1) {
  printInfo();
  initialize();
}

void XView::processFrameData(const FrameData& frame_data) {

  ++frame_number_;
  LOG(INFO) << "XView starts processing frame " << frame_number_ << ".";
  const RowVector3r origin = frame_data.getPose().getPosition().cast<real_t>();
  const Matrix3r rotation = frame_data.getPose().getRotationMatrix()
      .cast<real_t>();
  LOG(INFO) << "Associated robot pose:\n"
            << formatSE3(frame_data.getPose(), "\t\t", 3);

  const auto& timer = Locator::getTimer();
  timer->registerTimer("ProcessFrameData");
  timer->start("ProcessFrameData");

  // Generate a new semantic landmark pointer.
  SemanticLandmarkPtr landmark_ptr;

  // Extract semantics associated to the semantic image and pose.
  timer->registerTimer("SemanticLandmarkExtraction", "ProcessFrameData");
  timer->start("SemanticLandmarkExtraction");
  createSemanticLandmark(frame_data, landmark_ptr);
  timer->stop("SemanticLandmarkExtraction");

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

  timer->stop("ProcessFrameData");

  LOG(INFO) << "XView ended processing frame " << frame_number_ << ".";
}

const Graph& XView::getSemanticGraph() const {
  CHECK(Locator::getParameters()->getChildPropertyList("matcher")->
      getString("type") == "GRAPH")
  << "Function " << __FUNCTION__ << " can only be used when X-View runs the "
  << "'GRAPH' matcher type, currently using "
  << Locator::getParameters()->getChildPropertyList("matcher")->
      getString("type") << ".";

  const auto graph_matcher =
      std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_);

  CHECK_NOTNULL(graph_matcher.get());

  return graph_matcher->getGlobalGraph();
}

Graph& XView::getSemanticGraph() {
  CHECK(Locator::getParameters()->getChildPropertyList("matcher")->
      getString("type") == "GRAPH")
  << "Function " << __FUNCTION__ << " can only be used when X-View runs the "
  << "'GRAPH' matcher type, currently using "
  << Locator::getParameters()->getChildPropertyList("matcher")->
      getString("type") << ".";

  const auto graph_matcher =
      std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_);

  CHECK_NOTNULL(graph_matcher.get());

  return graph_matcher->getGlobalGraph();
}

void XView::writeGraphToFile() const {
  const std::string filename = getOutputDirectory() + "merged_" +
      PaddedInt(frame_number_, 5, '0') + ".dot";
  const auto graph_matcher =
      std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_);
  CHECK_NOTNULL(graph_matcher.get());
  writeToFile(graph_matcher->getGlobalGraph(), filename);
}

real_t XView::localizeGraph(const Graph& query_graph,
                          std::vector<x_view::PoseId> pose_ids,
                          SE3* pose) {

  // Get the existing global semantic graph before matching.
  const Graph& global_graph = getSemanticGraph();

  // Perform full matching getting similarity scores between query graph and
  // existing global semantic graph.
  GraphMatcher::GraphMatchingResult matching_result;

  // Extract the random walks of the query graph.
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

  GraphMatcher::IndexMatrixType& candidate_matches =
      matching_result.getCandidateMatches();

  const std::string score_type_str = Locator::getParameters()
      ->getChildPropertyList("matcher")->getString("vertex_similarity_score");
  VertexSimilarity::SCORE_TYPE score_type;
  if(score_type_str == "WEIGHTED")
    score_type = VertexSimilarity::SCORE_TYPE::WEIGHTED;
  else if(score_type_str == "SURFACE")
    score_type = VertexSimilarity::SCORE_TYPE::SURFACE;
  else
    CHECK(false) << "Unrecognized score type " << score_type_str << ".";
  std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_)
      ->computeSimilarityMatrix(
          random_walker, &similarity_matrix, &candidate_matches, score_type);

  // Filter matches with geometric consistency.
  if (Locator::getParameters()->getChildPropertyList("matcher")->getBoolean(
      "outlier_rejection")) {
    bool filter_success =
        std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_)
            ->filterMatches(query_graph, global_graph,
                            matching_result, &candidate_matches);
  }

  // Estimate transformation between graphs (Localization).
  GraphLocalizer graph_localizer;

  // Utility function to check if vertices are to be added to the localizer
  // or not.
  auto isValidCandidate = [&](const uint64_t index) -> bool {
    const auto& col_i = candidate_matches.col(index);
    for(int i = 0; i < col_i.rows(); ++i)
      if(col_i(i) != GraphMatcher::INVALID_MATCH_INDEX)
        return true;
    return false;
  };


  // Add relative pose-to-pose observations to the graph localizer, assuming
  // poses are consecutive.

  for (size_t i = 0u; i < pose_ids.size() - 1; ++i) {
    graph_localizer.addPosePoseMeasurement(pose_ids[i], pose_ids[i + 1]);
  }

  // Add all pose-to-vertex observations.
  const uint64_t num_query_vertices = boost::num_vertices(query_graph);

  for (uint64_t i = 0u; i < num_query_vertices; ++i) {
    if(isValidCandidate(i)) {
      const VertexProperty& vertex_property = query_graph[i];
      // Add a pose-node observation for each observer of node.
      for (size_t i_pose = 0u; i_pose < vertex_property.observers.size();
           ++i_pose) {
        graph_localizer.addPoseVertexMeasurement(
            vertex_property, vertex_property.observers[i_pose]);
      }
    }
  }

  // Add all vertex-to-vertex observations.
  for (uint64_t j = 0; j < num_query_vertices; ++j) {
    // Add all candidate matches.
    for(uint64_t i_id = 0; i_id < candidate_matches.rows(); ++i_id) {
      if(candidate_matches(i_id, j) == GraphMatcher::INVALID_MATCH_INDEX)
        continue;
      const uint64_t i = candidate_matches(i_id, j);
      const real_t similarity = similarity_matrix(i, j);
      const VertexProperty& vertex_query = query_graph[j];
      const VertexProperty& vertex_global = global_graph[i];
      graph_localizer.addVertexVertexMeasurement(vertex_query, vertex_global,
                                                 similarity);
    }
  }

  const real_t error = graph_localizer.localize(matching_result, query_graph,
                                                global_graph, pose);

  return error;
}

void XView::relabelGlobalGraphVertices(const x_view::real_t percentage,
                                       const uint64_t seed) {

  const auto& dataset = Locator::getDataset();

  Graph& graph = getSemanticGraph();
  const uint64_t num_vertices = boost::num_vertices(graph);
  const uint64_t num_relabeled =
      static_cast<uint64_t>(std::round(percentage *  num_vertices));

  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> label_generator(
      0, dataset->numSemanticClasses() -1);

  std::set<VertexDescriptor> vertices_to_be_relabeled;
  while(vertices_to_be_relabeled.size() != num_relabeled) {
    const VertexDescriptor random_vertex_descriptor =
        boost::random_vertex(graph, rng);
    vertices_to_be_relabeled.insert(random_vertex_descriptor);
  }

  LOG(INFO) << "Relabeling " << vertices_to_be_relabeled.size()
            << " vertices out of " << num_vertices << ".";

  for(const VertexDescriptor random_vertex_descriptor :
      vertices_to_be_relabeled) {
    int new_label = label_generator(rng);
    // Make sure the new label is different from the original one
    while(new_label == graph[random_vertex_descriptor].semantic_label) {
      new_label = label_generator(rng);
    }
    graph[random_vertex_descriptor].semantic_label = new_label;
    graph[random_vertex_descriptor].semantic_entity_name =
        dataset->label(new_label);
  }

  // Recompute the random walks of the global semantic graph.
  std::dynamic_pointer_cast<GraphMatcher>(descriptor_matcher_)
      ->recomputeGlobalRandomWalks();
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

      #if X_VIEW_USE_DOUBLE_PRECISION
      << " (DP)"
      #else
      << " (SP)"
      #endif

      << "\n\n" << dataset
      << "\n\tLandmark type:\t<" + landmark_parameters->getString("type") + ">"
      << "\n\tMatcher type: \t<" + matcher_parameters->getString("type") + ">"
      << "\n\tLocalizer type: \t<" + localizer_parameters->getString("type")
          + ">"
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
  const auto current_graph_landmark =
      std::dynamic_pointer_cast<GraphLandmark>(semantics_a);
  CHECK_NOTNULL(current_graph_landmark.get());

  const auto current_graph_descriptor =
      std::dynamic_pointer_cast<const GraphDescriptor>
          (current_graph_landmark->getDescriptor());

  const auto& current_graph =
      current_graph_descriptor->getDescriptor();

  const auto graph_matching_result = std::dynamic_pointer_cast
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
