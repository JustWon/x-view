#include <x-view_core/x-view.h>
#include <x-view_core/x-view_semantics.h>

namespace x_view {

    XView::XView(XViewParams &params) : params_(params) {

        // create a factory object which is responsible for generating new semantic landmark observations
        semantic_factory_.setSemanticLandmarkType(params_.semantic_landmark_type_);
    }

    XView::~XView() {};

    void XView::extractSemanticsFromImage(const cv::Mat &image, const SE3 &pose,
                                          std::shared_ptr<XViewSemantics> &semantics_out) {

        // TODO: preprocess image and pose

        // create the actual landmark representation
        semantic_factory_.createSemanticLandmark(image, pose, semantics_out);

        // TODO: add the semantics_out landmark to the database
    }

    void XView::matchSemantics(const XViewSemantics &semantics_a, Eigen::MatrixXd &matches) {
        // TODO: match input semantics against semantics_db_ by doing a sort of loop and calling "this->semantics_db_[i].match(semantics_a)".
        CHECK(false) << "Not implemented.";
    }

    void XView::filterMatches(const XViewSemantics &semantics_a, Eigen::MatrixXd &matches) {
        // TODO: filter matches, e.g., with geometric verification.
        CHECK(false) << "Not implemented.";
    }

    void XView::mergeSemantics(const XViewSemantics &semantics_a, const Eigen::MatrixXd &matches) {
        // TODO: Merge semantics with semantics_db_ if dominant matches,
        // otherwise add semantics as new instance to semantics_db_.
        // TODO: use filterMatches function before merging.
        CHECK(false) << "Not implemented.";
    }

    void XView::cleanDatabase() {
        // TODO: sweep over semantics_db_ to match and merge unconnected semantics.
        CHECK(false) << "Not implemented.";
    }

}
