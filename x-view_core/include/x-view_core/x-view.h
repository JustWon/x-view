#ifndef X_VIEW_X_VIEW_H_
#define X_VIEW_X_VIEW_H_

#include <x-view_core/types.h>
#include <x-view_core/x-view_semantics.h>

#include <opencv2/core/core.hpp>

#include <memory>

namespace x_view {

    struct XViewParams {
        // TODO: define if we want to use BoS or other structure as semantic landmark representation

    };

    class XView {

    public:

        XView() {};

        explicit XView(XViewParams &params);

        ~XView();

        /**
         * \brief Extract semantic descriptor from semantics image.
         * \param image image containing semantic segmentation
         * \param pose current pose of the robot
         * \param semantics_out representation of semantic entities, either a BoS, or a more complex representation
         */
        void extractSemanticsFromImage(const cv::Mat &image, const SE3 &pose,
                                       std::unique_ptr<XViewSemantics> &semantics_out);

        /// \brief Match semantics instance to database and return score.
        void matchSemantics(const XViewSemantics &semantics_a, Eigen::MatrixXd &matches);

        /// \brief Filter matches, e.g., geometric verification etc.
        void filterMatches(const XViewSemantics &semantics_a, Eigen::MatrixXd &matches);

        /// \brief Merge semantics instance into database according to matches.
        void mergeSemantics(const XViewSemantics &semantics_a, const Eigen::MatrixXd &matches);

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
        std::vector<XViewSemantics*> semantics_db_;
    }; // XView

}
#endif /* X_VIEW_X_VIEW_H_ */
