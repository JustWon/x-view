#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

namespace x_view {

// Set default score type
VertexSimilarity::SCORE_TYPE VertexSimilarity::current_score_type_ =
    VertexSimilarity::SCORE_TYPE::HARD;

VertexSimilarity::ScoreFunctionType VertexSimilarity::score =
    VertexSimilarity::getScoreFunction();

void VertexSimilarity::setScoreType(
    const VertexSimilarity::SCORE_TYPE score_type) {
  VertexSimilarity::current_score_type_ = score_type;
  VertexSimilarity::score = VertexSimilarity::getScoreFunction();
}

VertexSimilarity::ScoreFunctionType VertexSimilarity::getScoreFunction() {
  switch (VertexSimilarity::current_score_type_) {
    case VertexSimilarity::SCORE_TYPE::HARD: {
      return VertexSimilarity::score_hard;
    }
    case VertexSimilarity::SCORE_TYPE::WEIGHTED: {
      return VertexSimilarity::score_weighted;
    }
    default: {
      CHECK(false) << "Score type passed to " << __FUNCTION__
                   << " is unrecognized.";
    }
  }
}

const float VertexSimilarity::score_hard(const RandomWalker::WalkMap& node1,
                                         const RandomWalker::WalkMap& node2) {
  int score = 0;
  int normalization = 0;
  for (const auto& walk1 : node1) {
    const int walk1_id = walk1.first;
    const int walk1_multiplicity = walk1.second.multiplicity_;
    for (const auto& walk2 : node2) {
      const int walk2_id = walk2.first;
      const int walk2_multiplicity = walk2.second.multiplicity_;

      const int multiplicity = walk1_multiplicity * walk2_multiplicity;
      normalization += multiplicity;
      bool are_walks_identical = (walk1_id == walk2_id);
      if (are_walks_identical) {
        score += multiplicity;
      }
    }
  }
  return float(score) / (normalization+1);
}

const float VertexSimilarity::score_weighted(const RandomWalker::WalkMap& node1,
                                             const RandomWalker::WalkMap& node2) {
  // TODO implement this
  CHECK(false) << "Function " << __FUNCTION__ << " is not implemented yet.";
}

}