#ifndef X_VIEW_TEST_WALK_MATCHING_H
#define X_VIEW_TEST_WALK_MATCHING_H

#include <algorithm>
#include <chrono>
#include <random>
#include <unordered_map>
#include <vector>

typedef unsigned char Label_type;

typedef std::vector<Label_type> Walk;

typedef int ID_type;

struct WalkCount {
  const Walk walk_;
  int multiplicity_;

  WalkCount(const Walk& walk)
      : walk_(walk),
        multiplicity_(1) {
  }

  WalkCount& operator++() { // ++WalkCount
    ++this->multiplicity_;
    return *this;
  }
};

typedef std::unordered_map<ID_type, WalkCount> WalkMap;

typedef std::mt19937 Engine;

typedef std::uniform_int_distribution<Label_type> Distribution;

class ExampleNode {
 public:
  ExampleNode(const int num_walks, const int walk_length,
              const Label_type num_classes, const int seed);

 private:

  static const ID_type walkToID(const Walk& walk, const int num_classes);

  static Walk generateRandomWalkOfLength(const int length, Engine& engine,
                                         Distribution& dist);

  void generateRandomWalks();

  friend std::ostream& operator<<(std::ostream& out, const ExampleNode& node);

 private:
  const int num_walks_;
  const int walk_length_;
  const Label_type num_classes_;
  const int seed_;

  Engine engine_;
  Distribution distribution_;

  WalkMap walk_map_;

};

#endif //X_VIEW_TEST_WALK_MATCHING_H
