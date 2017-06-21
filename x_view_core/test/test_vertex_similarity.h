#ifndef X_VIEW_TEST_VERTEX_SIMILARITY_H
#define X_VIEW_TEST_VERTEX_SIMILARITY_H

#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

using namespace x_view;

namespace x_view_test {

class VertexSimilarityTest {

 public:

  VertexSimilarityTest()
      : vertices_left_(0),
        vertices_right_(0),
        expected_score_(0),
        score_type_(VertexSimilarity::SCORE_TYPE::WEIGHTED) {
  }

  void setScoreType(const VertexSimilarity::SCORE_TYPE score_type) {
    VertexSimilarity::setScoreType(score_type);
    score_type_ = score_type;
  }

  void addWalkMap(const RandomWalker::WalkMap& walk_left,
                  const RandomWalker::WalkMap& walk_right,
                  const float expected_score);

  void run() const;
  void checkExpectedSimilarity() const;
  void checkSymmetry() const;

  static RandomWalker::WalkMap generateWalkMap(const std::vector<std::vector<int>>& random_walks);

 private:
  std::vector<RandomWalker::WalkMap> vertices_left_;
  std::vector<RandomWalker::WalkMap> vertices_right_;
  std::vector<float> expected_score_;
  VertexSimilarity::SCORE_TYPE score_type_;
};

void testScoreSymmetry();


}

#endif //X_VIEW_TEST_VERTEX_SIMILARITY_H
