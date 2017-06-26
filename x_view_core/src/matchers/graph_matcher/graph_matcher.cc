#include <x_view_core/matchers/graph_matcher/graph_matcher.h>

#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/matchers/graph_matcher/similarity_plotter.h>
#include <x_view_core/x_view_locator.h>

namespace x_view {

GraphMatcher::MaxSimilarityMatrixType
GraphMatcher::GraphMatchingResult::computeMaxSimilarityColwise() const {
  // Compute the index of max element per col of similarity matrix.
  const long cols = similarity_matrix_.cols();
  const long rows = similarity_matrix_.rows();

  MaxSimilarityMatrixType max_similarity_colwise(rows, cols);
  max_similarity_colwise.setZero();

  // max_indices_per_col[j] contains a vector of indices 'i' such that
  // all similarity_matrix(i,j) are maximal and not zero in the j-th column.
  std::vector<std::vector<int> > max_indices_per_col(cols);
  // Iterate over all columns.
  for (int j = 0; j < cols; ++j) {
    float max_val = similarity_matrix_.col(j).maxCoeff();
    if (max_val > 0.f)
      for (int i = 0; i < rows; ++i) {
        if (similarity_matrix_(i, j) == max_val)
          max_indices_per_col[j].push_back(i);
      }
  }

  // Set the max elements to true.
  for (int j = 0; j < cols; ++j) {
    for (int k = 0; k < max_indices_per_col[j].size(); ++k)
      max_similarity_colwise(max_indices_per_col[j][k], j) = true;
  }

  return max_similarity_colwise;
}

GraphMatcher::MaxSimilarityMatrixType
GraphMatcher::GraphMatchingResult::computeMaxSimilarityRowwise() const {
  // Compute the index of max element per row of similarity matrix.
  const long cols = similarity_matrix_.cols();
  const long rows = similarity_matrix_.rows();

  MaxSimilarityMatrixType max_similarity_rowwise(rows, cols);
  max_similarity_rowwise.setZero();

  // max_indices_per_row[i] contains a vector of indices 'j' such that
  // all similarity_matrix(i,j) are maximal and not zero in the i-th row.
  std::vector<std::vector<int> > max_indices_per_row(rows);
  // Iterate over all rows.
  for (int i = 0; i < rows; ++i) {
    float max_val = similarity_matrix_.row(i).maxCoeff();
    if (max_val > 0.f)
      for (int j = 0; j < cols; ++j) {
        if (similarity_matrix_(i, j) == max_val)
          max_indices_per_row[i].push_back(j);
      }
  }

  // Set the max elements to true.
  for (int i = 0; i < rows; ++i) {
    for (int k = 0; k < max_indices_per_row[i].size(); ++k)
      max_similarity_rowwise(i, max_indices_per_row[i][k]) = true;
  }

  return max_similarity_rowwise;
}

GraphMatcher::GraphMatcher() {
  const auto& parameters = Locator::getParameters();
  const auto& matcher_parameters = parameters->getChildPropertyList("matcher");
  const std::string score_type =
      matcher_parameters->getString("vertex_similarity_score");
  if(score_type == "WEIGHTED")
    vertex_similarity_score_type_ = VertexSimilarity::SCORE_TYPE::WEIGHTED;
  else if(score_type == "SURFACE")
    vertex_similarity_score_type_ = VertexSimilarity::SCORE_TYPE::SURFACE;
  else
    LOG(ERROR) << "Unrecognized vertex score type <" << score_type << ">.";

  const std::string random_walk_sampling_type =
      matcher_parameters->getString("random_walk_sampling_type");
  if(random_walk_sampling_type == "UNIFORM")
    random_walker_params_.random_sampling_type =
        RandomWalkerParams::SAMPLING_TYPE::UNIFORM;
  else if(random_walk_sampling_type == "AVOIDING") {
    random_walker_params_.random_sampling_type =
    RandomWalkerParams::SAMPLING_TYPE::AVOIDING;
  } else
    LOG(ERROR) << "Unrecognized random walker sampling type <"
               << random_walk_sampling_type << ">.";

  random_walker_params_.num_walks =
      matcher_parameters->getInteger("num_walks");
  random_walker_params_.walk_length =
      matcher_parameters->getInteger("walk_length");

  SimilarityPlotter::setColormap(cv::COLORMAP_OCEAN);
}

GraphMatcher::GraphMatcher(const RandomWalkerParams& random_walker_params,
                           const VertexSimilarity::SCORE_TYPE score_type)
    : random_walker_params_(random_walker_params),
      vertex_similarity_score_type_(score_type) {
  SimilarityPlotter::setColormap(cv::COLORMAP_OCEAN);
}

GraphMatcher::~GraphMatcher() {
}

AbstractMatcher::MatchingResultPtr GraphMatcher::match(const SemanticLandmarkPtr& query_landmark) {

  // Cast the SemanticLandmarkPtr to a GraphLandmarkPtr.
  const auto graph_landmark_ptr =
      std::dynamic_pointer_cast<const GraphLandmark>(query_landmark);

  CHECK(graph_landmark_ptr != nullptr) << "Impossible to cast 'query_landmark' "
      "to a 'GraphLandmark' pointer. Be sure the passed 'SemanticLandmarkPtr' "
      "points to an instance of 'GraphLandmark'.";

  // Cast the descriptor associated to the graph_landmark_ptr to a
  // GraphDescriptor.
  const auto graph_descriptor =
      std::dynamic_pointer_cast<const GraphDescriptor>(
          graph_landmark_ptr->getDescriptor());

  // Perform checks related to the cast.
  CHECK(graph_descriptor != nullptr) << "Impossible to cast descriptor "
      "associated to graph_landmark_ptr to a 'const GraphDescriptor'";

  // Extract the representation of the descriptor from the graph_descriptor
  // object.
  const Graph& query_semantic_graph = graph_descriptor->getDescriptor();

  // Delegate the call to function handling directly a graphs as input.
  return match(query_semantic_graph);
}

AbstractMatcher::MatchingResultPtr GraphMatcher::match(
    const Graph& query_semantic_graph) {

  CHECK(boost::num_vertices(global_semantic_graph_) > 0)
  << "You are trying to match a graph landmark using an uninitialized "
  << "graph matcher: the global_semantic_graph_ has 0 vertices, thus a "
  << "match is not possible. "
  << "Make sure to call 'GraphMatcher::addDescriptor' during the first "
  << "frame to simply add the first graph to the descriptor without "
  << "performing any match.";

  // Extract the random walks of the graph.
  RandomWalker random_walker(query_semantic_graph, random_walker_params_);
  random_walker.generateRandomWalks();

  // Create a matching result pointer which will be returned by this
  // function which stores the similarity matrix.
  auto matchingResult = std::make_shared<GraphMatchingResult>();

  SimilarityMatrixType& similarity_matrix =
      matchingResult->getSimilarityMatrix();

  computeSimilarityMatrix(random_walker, &similarity_matrix,
                          vertex_similarity_score_type_);

  // Return the matching result filled with the matches.
  return matchingResult;

}

void GraphMatcher::addDescriptor(const ConstDescriptorPtr& descriptor) {
  const Graph& graph = std::dynamic_pointer_cast<const GraphDescriptor>
      (descriptor)->getDescriptor();
  addDescriptor(graph);
}

void GraphMatcher::addDescriptor(const Graph& graph) {
  CHECK(boost::num_vertices(graph) > 0)
  << "You are adding a graph with 0 vertices to the GraphMatcher.";
  RandomWalker random_walker(graph, random_walker_params_);
  random_walker.generateRandomWalks();

  global_semantic_graph_.clear();
  global_semantic_graph_ = graph;

  global_walk_map_vector_.clear();
  global_walk_map_vector_ = random_walker.getMappedWalks();
}

LandmarksMatcherPtr GraphMatcher::create() {
  return std::make_shared<GraphMatcher>();
}

LandmarksMatcherPtr GraphMatcher::create(const RandomWalkerParams& random_walker_params,
                                         const VertexSimilarity::SCORE_TYPE score_type) {
  return std::make_shared<GraphMatcher>(GraphMatcher(random_walker_params,
                                                     score_type));
}

void GraphMatcher::computeSimilarityMatrix(const RandomWalker& random_walker,
                                           Eigen::MatrixXf* similarity_matrix,
                                           const VertexSimilarity::SCORE_TYPE score_type) const {
  CHECK_NOTNULL(similarity_matrix);

  // Extract the random walks from the RandomWalker passed as argument.
  const std::vector<RandomWalker::WalkMap>& query_walk_map_vector =
      random_walker.getMappedWalks();

  // Extract the graph over which the random walks are defined.
  const Graph& query_graph = random_walker.graph();

  // Set the similarity score type to be used.
  VertexSimilarity::setScoreType(score_type);

  const unsigned long num_global_vertices =
      boost::num_vertices(global_semantic_graph_);
  const unsigned long num_query_vertices = boost::num_vertices(query_graph);

  similarity_matrix->resize(num_global_vertices, num_query_vertices);
  similarity_matrix->setZero();

  // Fill up dense similarity matrix.
  for (unsigned long i = 0; i < num_global_vertices; ++i) {
    const auto& vertex_d_i = boost::vertex(i, global_semantic_graph_);
    const auto& vertex_p_i = global_semantic_graph_[vertex_d_i];
    const auto& mapped_walks_i = global_walk_map_vector_[i];
    for (unsigned long j = 0; j < num_query_vertices; ++j) {
      const auto& vertex_d_j = boost::vertex(j, query_graph);
      const auto& vertex_p_j = query_graph[vertex_d_j];
      // Score is only nonzero if the source vertex has same semantic label.
      if (vertex_p_i.semantic_label == vertex_p_j.semantic_label) {
        const auto& mapped_walks_j = query_walk_map_vector[j];
        const float similarity =
            VertexSimilarity::score(mapped_walks_i, mapped_walks_j);
        similarity_matrix->operator()(i, j) = similarity;
      }
    }
  }
}

}

