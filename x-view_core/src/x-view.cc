#include <x-view_core/x-view.h>

namespace x_view {

    XView::XView(XViewParams &params) : params_(params) {}

    XView::~XView() {};

    void XView::extractSemanticsFromImage(const cv::Mat &image, const SE3 &pose,
                                          XViewSemantics *semantics_out) {
        // TODO: build semantic descriptor from smenatics image input.
        CHECK(false) << "Not implemented.";
    }

    void XView::matchSemantics(const XViewSemantics &semantics_a,
                               Eigen::MatrixXd *matches) {
        // TODO: match input semantics against semantics_db_.
        CHECK(false) << "Not implemented.";
    }

    void XView::filterMatches(const XViewSemantics &semantics_a,
                              Eigen::MatrixXd *matches) {
        // TODO: filter matches, e.g., with geometric verification.
        CHECK(false) << "Not implemented.";
    }

    void XView::mergeSemantics(const XViewSemantics &semantics_a,
                               const Eigen::MatrixXd &matches) {
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
