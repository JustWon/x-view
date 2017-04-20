#ifndef X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H
#define X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H

#include <x_view_core/x_view_types.h>

namespace x_view {

/**
 * \brief An interface each landmark-matcher must implement
 */
class AbstractLandmarksMatcher {

 public:
  AbstractLandmarksMatcher() {}
  virtual ~AbstractLandmarksMatcher() {}

  /**
   * \brief Each landmark matcher returns a different type of matching result.
   * For this reason, each class implementing the AbstractLandmarksMatcher
   * interface also has to define an ReturnType for the match() function
   * which contains information on the matching result.
   */
  class AbstractMatchingResult {
   public:
    AbstractMatchingResult();
    virtual ~AbstractMatchingResult() = 0;
  };

  typedef std::shared_ptr<AbstractMatchingResult> MatchingResultPtr;

  /**
   * \brief Add a landmark to the matcher, which will update its internal
   * representation of the previously observed landmarks
   * \param landmark const reference to semantic landmark pointer
   * \note landmark is a pointer to the AbstractSemanticLandmark class; each
   * class implementing this interface is responsible for casting that
   * parameter to the one expected by the concrete class.
   */
  virtual void addLandmark(const SemanticLandmarkPtr& landmark) = 0;

  /**
   * \brief Computes a match between the query landmark passed as parameter
   * and the features internally stored
   * \param queryLandmark const reference to semantic landmark pointer
   * \param matchingResult reference to matching result pointer, filled by
   * this function
   */
  virtual void match(const SemanticLandmarkPtr& queryLandmark,
                     MatchingResultPtr& matchingResult) = 0;

};

}

#endif //X_VIEW_ABSTRACT_LANDMARKS_MATCHER_H
