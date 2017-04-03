#ifndef X_VIEW_X_VIEW_H_
#define X_VIEW_X_VIEW_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>

namespace x_view {
    typedef kindr::minimal::QuatTransformationTemplate<double> SE3;

    struct XViewParams {
    }; // struct XViewParams

    struct XViewSemantics {
        SE3 pose;
        // TODO: Define structure of semantics graph.
        gtsam::NonlinearFactorGraph factor_graph;

        XViewSemantics() {};

        ~XViewSemantics() {};
    }; // struct XViewSemantics

    class XView {

    public:
        XView() {};

        explicit XView(XViewParams &params);

        ~XView();

        /// \brief Extract semantic descriptor from semantics image.
        void extractSemanticsFromImage(const cv::Mat &image, const SE3 &pose,
                                       XViewSemantics *semantics_out);

        /// \brief Match semantics instance to database and return score.
        void matchSemantics(const XViewSemantics &semantics_a,
                            Eigen::MatrixXd *matches);

        /// \brief Filter matches, e.g., geometric verification etc.
        void filterMatches(const XViewSemantics &semantics_a,
                           Eigen::MatrixXd *matches);

        /// \brief Merge semantics instance into database according to matches.
        void mergeSemantics(const XViewSemantics &semantics_a,
                            const Eigen::MatrixXd &matches);

        /// \brief Clean database by doing full semantics matching.
        void cleanDatabase();

        // TODO: Add further functions.

    private:
        // Set the parameters.
        void setParameters(const XViewParams &params) { params_ = params; }

        // TODO: Add further setters / getters where necessary.

        // Parameters.
        XViewParams params_;
        // Semantics database.
        std::vector<XViewSemantics> semantics_db_;
    }; // XView

}
#endif /* X_VIEW_X_VIEW_H_ */
