#ifndef X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H
#define X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H

#include <x_view_core/x_view_types.h>

namespace x_view {

/**
 * \brief An interface each landmark-matcher must implement
 */
class AbstractLandmarksMatcher {

 public:
  AbstractLandmarksMatcher(){}
  virtual ~AbstractLandmarksMatcher(){}

  /**
   * \brief Add a landmark to the matcher, which will update its internal
   * representation of the previously observed landmarks
   * \param landmark const reference to semantic landmark pointer
   * \note landmark is a pointer to the AbstractSemanticLandmark class; each
   * class implementing this interface is responsible for casting that
   * parameter to the one expected by the concrete class.
   */
  virtual void addLandmark(const SemanticLandmarkPtr& landmark) = 0;
};

}

#endif //X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H
