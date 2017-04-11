#ifndef X_VIEW_X_VIEW_H_
#define X_VIEW_X_VIEW_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/semantic_landmark_factory.h>
#include <x_view_core/abstract_semantic_landmark.h>

#include <opencv2/core/core.hpp>

#include <memory>

namespace x_view {

struct XViewParams {
  /// semantic landmark type to be used as landmark representation,
  /// this is used as argument for the landmark factory and all landmarks generated by XView will be of the specified type
  std::string semantic_landmark_type_string_;

};

class XView {

 public:

  XView() {};

  /**
   * \brief Constructs the XView object which handles the semantic SLAM problem
   * \param params struct containing all parameters used by XView
   */
  explicit XView(XViewParams& params);

  ~XView();

  /**
   * \brief x_view processes new landmark associated to image and pose
   * \param image semantic image representing landmark
   * \param pose robot's pose
   */
  void process(const cv::Mat& image, const SE3& pose);

  /**
   * \brief Extract semantic descriptor from semantics image.
   * \param image image containing semantic segmentation
   * \param pose current pose of the robot
   * \param semantics_out representation of semantic entities, either a BoS, or a more complex representation
   */
  void extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                 SemanticLandmarkPtr& semantics_out);

  /// \brief Match semantics instance to database and return score.
  void matchSemantics(const SemanticLandmarkPtr& semantics_a,
                      Eigen::MatrixXd& matches);

  /// \brief Filter matches, e.g., geometric verification etc.
  void filterMatches(const SemanticLandmarkPtr& semantics_a,
                     Eigen::MatrixXd& matches);

  /// \brief Merge semantics instance into database according to matches.
  void mergeSemantics(const SemanticLandmarkPtr& semantics_a,
                      const Eigen::MatrixXd& matches);

  /// \brief Clean database by doing full semantics matching.
  void cleanDatabase();

  // TODO: Add further functions.

 private:
  // Set the parameters.
  void setParameters(const XViewParams& params) { params_ = params; }

  // TODO: Add further setters / getters where necessary.

  // semantic landmark properties and factory
  SemanticLandmarkType semantic_landmark_type_;
  SemanticLandmarkFactory semantic_factory_;

  // Parameters.
  XViewParams params_;

  // Semantics database.
  std::vector<SemanticLandmarkPtr> semantics_db_;
}; // XView

}
#endif /* X_VIEW_X_VIEW_H_ */
