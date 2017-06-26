#ifndef X_VIEW_X_VIEW_H_
#define X_VIEW_X_VIEW_H_

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/landmarks/semantic_landmark_factory.h>
#include <x_view_core/matchers/abstract_matcher.h>
#include <x_view_core/parameters/parameters.h>
#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <memory>

namespace x_view {

/**
 * \brief The XView class is responsible for performing semantic SLAM.
 * \details The class operates on abstract types through pointers. This
 * allows XView to be functional with different types of features/landmarks
 * with no need to change the existing code.
 */
class XView {

 public:

  /**
   * \brief Constructs the XView object which handles the semantic SLAM problem.
   */
   XView();

  /**
   * \brief XView processes new landmark associated to a semantic image and
   * a robot's pose.
   * \param image Semantic image representing landmark.
   * \param pose Robot's pose.
   */
  void processSemanticImage(const cv::Mat& image, const SE3& pose);

 private:
  /// \brief Prints XView info.
  void printInfo() const;

  /// \brief Initializes all variables based on the parameters located in
  /// Locator::getParameters().
  void initialize();

  /// \brief Initializes the landmark factory based on the value retrieved
  /// from Locator::getParameters()->getChildPropertyList("landmark")->getString("type")
  void initializeLandmarkFactory();

  /// \brief Initializes the matchert based on the value retrieved
  /// from Locator::getParameters()->getChildPropertyList("matcher")->getString("type")
  void initializeMatcher();

  //=======================================================================//
  //        FUNCTIONS CALLED BY 'processSemanticImage' FUNCTION            //
  //=======================================================================//

  /**
   * \brief Extract semantic descriptor from semantics image.
   * \param image Image containing semantic information in the first channel.
   * \param pose Current pose of the robot.
   * \param semantics_out Generated landmark.
   * \details Depending on the XView parameters passed to the class
   * constructor, the dynamic type of the object pointed by semantics_out
   * will be different.
   */
  void extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                 SemanticLandmarkPtr& semantics_out);

  /// \brief Match semantics instance to database and return score.
  void matchSemantics(const SemanticLandmarkPtr& semantics_a,
                      AbstractMatcher::MatchingResultPtr& matching_result);

  /// \brief Filter matches, e.g., geometric verification etc.
  void filterMatches(const SemanticLandmarkPtr& semantics_a,
                     AbstractMatcher::MatchingResultPtr& matching_result);

  /// \brief Merge semantics instance into database according to matches.
  void mergeSemantics(const SemanticLandmarkPtr& semantics_a,
                      AbstractMatcher::MatchingResultPtr& matching_result);

  /// \brief Clean database by doing full semantics matching.
  void cleanDatabase();

  //=======================================================================//
  //                        CLASS MEMBER VARIABLES                         //
  //=======================================================================//

  /// \brief Semantic landmark factory which generates instances of semantic
  /// landmarks.
  SemanticLandmarkFactory semantic_landmark_factory_;

  /// \brief Semantic landmark matcher computes a matching between a new
  /// semantic landmark and the ones previously added to it.
  LandmarksMatcherPtr descriptor_matcher_;

  /// \brief Vector of semantic landmarks pointers visited by XView.
  std::vector<SemanticLandmarkPtr> semantics_db_;

  /// \brief Current number of frames processed by XView.
  unsigned long frame_number_;

}; // XView

}
#endif //X_VIEW_X_VIEW_H_
