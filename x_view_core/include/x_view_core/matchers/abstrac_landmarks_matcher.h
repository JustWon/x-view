#ifndef X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H
#define X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H

namespace x_view {

/**
 * \brief An interface each landmark-matcher must implement
 */
class AbstractLandmarksMatcher {

 public:
  AbstractLandmarksMatcher(){}
  virtual ~AbstractLandmarksMatcher(){}
};

}

#endif //X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H
