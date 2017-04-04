#include <x-view_node/x-view_worker.h>

#include <x-view_core/semantic_landmark_factory.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

namespace x_view_ros {

    XViewWorker::XViewWorker(ros::NodeHandle &n) : nh_(n) {
        // FIXME: who initializes 'params_'?
        semantics_image_sub_ = nh_.subscribe(params_.semantics_image_topic, 1,
                                             &XViewWorker::semanticsImageCallback,
                                             this);
        getParameters();
        x_view_ = x_view::XView(params_.x_view_params);
    }

    XViewWorker::~XViewWorker() {
    }

    void XViewWorker::semanticsImageCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = image_ptr->image;
            // TODO: Process image using x-view functions.
            CHECK(false) << "Not implemented.";
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR_STREAM("Could not convert from " << msg->encoding.c_str() << " to 'bgr8'.");
        }
    }

    void XViewWorker::getParameters() {

        std::string semanticLandmarkTypeString;
        // TODO: how do we get the parameters? should there be a 'hierarchical' structure in the yaml file to follow
        /* e.g. core:
         *          landmarks:
         *              type: "bos"
         *              num: 10
         *              ...
         *          ...
         *      worker:
         *          topics:
         *              semantic_images = "semantic_image_topic"
         *              ...
         *          ...
         *      ....
         */
        if (nh_.getParam("semanticLandmarkType", semanticLandmarkTypeString)) {
            ROS_INFO_STREAM("XView is using the following semantic landmark type:" << semanticLandmarkTypeString);
            if (semanticLandmarkTypeString.compare("bos") == 0) {
                params_.x_view_params.semantic_landmark_type_ = x_view::SemanticLandmarkFactory::SEMANTIC_LANDMARK_TYPE::BOS;
            } else if (semanticLandmarkTypeString.compare("graph") == 0) {
                params_.x_view_params.semantic_landmark_type_ = x_view::SemanticLandmarkFactory::SEMANTIC_LANDMARK_TYPE::GRAPH;
            } else {
                ROS_ERROR_STREAM("Parameter associated to 'semanticLandmarkType' is unknown:\n\tgiven: "
                                         << semanticLandmarkTypeString << "\n\tvalid: {'bos', 'graph'}");
            }

        } else {
            ROS_ERROR_STREAM("Failed to get param 'semanticLandmarkType'");
        }

        // TODO: read in other parameters from node.
    }

}
