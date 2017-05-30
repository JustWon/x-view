#include "test_walk_matching.h"

#include <glog/logging.h>

#include <iostream>

ExampleNode::ExampleNode(const int num_walks, const int walk_length,
                         const Label_type num_classes, const int seed)
    : num_walks_(num_walks),
      walk_length_(walk_length),
      num_classes_(num_classes),
      seed_(seed),
      engine_(seed),
      distribution_(0, num_classes_ - 1) {
  generateRandomWalks();
}

const ID_type ExampleNode::walkToID(const Walk& walk, const int num_classes) {
  ID_type id = 0;
  ID_type mult = 1;
  for (const auto val : walk) {
    id += mult * val;
    mult *= num_classes;
  }
  return id;
}

Walk ExampleNode::generateRandomWalkOfLength(const int length, Engine& engine,
                                             Distribution& dist) {
  Walk res(length);
  std::generate(res.begin(), res.end(), [&]() { return dist(engine); });
  return res;
}

void ExampleNode::generateRandomWalks() {
  walk_map_.reserve(num_walks_);
  for (int i = 0; i < num_walks_; ++i) {
    const auto walk =
        ExampleNode::generateRandomWalkOfLength(walk_length_, engine_,
                                                distribution_);
    const auto walk_ID = ExampleNode::walkToID(walk, num_classes_);
    WalkMap::iterator found_position = walk_map_.find(walk_ID);
    if (found_position == walk_map_.end())
      walk_map_.insert({walk_ID, WalkCount(walk)});
    else
      ++(found_position->second);
  }
}

std::ostream& operator<<(std::ostream& out, const ExampleNode& node) {
  out << "Num walks: " << node.num_walks_ << std::endl;
  out << "Walk length: " << node.walk_length_ << std::endl;
  out << "Num classes: " << static_cast<int>(node.num_classes_) << std::endl;
  out << "Seed: " << node.seed_ << std::endl;

  out << "Walks: " << std::endl;
  for (const auto& walk : node.walk_map_) {
    out << "ID: " << walk.first;
    out << ", labels: ";
    for (const auto val : walk.second.walk_) {
      out << static_cast<int>(val) << ", ";
    }
    out << " Multiplicity: " << walk.second.multiplicity_ << std::endl;
  }
  return out;
}