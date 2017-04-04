#ifndef X_VIEW_SEMANTIC_FACTORY_H
#define X_VIEW_SEMANTIC_FACTORY_H

#include <x-view_core/types.h>

#include <opencv2/core/core.hpp>

namespace x_view {

    // forward declaration
    struct XViewSemantics;

    /**
     * \brief Class responsible for creating new landmarks
     */
    class XViewSemanticFactory {

    public:

        enum SEMANTIC_LANDMARK_TYPE {
            UNDEFINED_SEMANTIC_LANDMARK_TYPE = -1,  // factory is initialized with this landmark type, it will raise an exception if not changed
            BOS,    /// Bag of Semantics approach
            GRAPH,  /// Graph approach
            NUM_SEMANTIC_LANDMARK_TYPES
        };

        /**
         * \brief Creates a factory object which will create semantic landmarks defined by the passed argument
         * \param type enum specifying the semantic landmark type to be constructed
         */
        explicit XViewSemanticFactory(SEMANTIC_LANDMARK_TYPE type = SEMANTIC_LANDMARK_TYPE::UNDEFINED_SEMANTIC_LANDMARK_TYPE)
                : semanticLandmarkType_(type) {}

        void setSemanticLandmarkType(const SEMANTIC_LANDMARK_TYPE type) {
            semanticLandmarkType_ = type;
        }

        SEMANTIC_LANDMARK_TYPE getSemanticLandmarkType() const {
            return semanticLandmarkType_;
        }

        /**
         * \brief Function exposed to the user to create new semantic landmark objects
         * \param image Image containing semantic segmentation
         * \param pose  3D pose of the robot associated to the image
         * \param landmark pointer to abstract base landmark class which is filled up with a concrete landmark type
         */
        void createSemanticLandmark(const cv::Mat &image, const SE3 &pose, std::shared_ptr<XViewSemantics> &landmark);

    private:
        SEMANTIC_LANDMARK_TYPE semanticLandmarkType_;

    };
}

#endif //X_VIEW_SEMANTIC_FACTORY_H
