#ifndef X_VIEW_PARSER_H
#define X_VIEW_PARSER_H

#include <x_view_core/parameters/parameters.h>

#include <ros/ros.h>

#include <memory>

namespace x_view_ros {

void addString(const ros::NodeHandle& nh, const std::string& ros_key,
               const std::string& param_key,
               std::unique_ptr<x_view::Parameters>& parameters,
               const std::string& default_string = "");

void addInt(const ros::NodeHandle& nh, const std::string& ros_key,
            const std::string& param_key,
            std::unique_ptr<x_view::Parameters>& parameters,
            const int default_int = 0);

void addFloat(const ros::NodeHandle& nh, const std::string& ros_key,
              const std::string& param_key,
              std::unique_ptr<x_view::Parameters>& parameters,
              const float default_float = 0.f);

void addBool(const ros::NodeHandle& nh, const std::string& ros_key,
             const std::string& param_key,
             std::unique_ptr<x_view::Parameters>& parameters,
             const bool default_bool = false);

std::unique_ptr<x_view::Parameters> parseParameters(const ros::NodeHandle& nh);

std::unique_ptr<x_view::Parameters> parseDataset(const ros::NodeHandle& nh);

std::unique_ptr<x_view::Parameters> parseLandmark(const ros::NodeHandle& nh);
std::unique_ptr<x_view::Parameters> parseHistogramLandmark(const ros::NodeHandle& nh);
std::unique_ptr<x_view::Parameters> parseORBLandmark(const ros::NodeHandle& nh);
std::unique_ptr<x_view::Parameters> parseSIFTLandmark(const ros::NodeHandle& nh);
std::unique_ptr<x_view::Parameters> parseSURFLandmark(const ros::NodeHandle& nh);
std::unique_ptr<x_view::Parameters> parseGraphLandmark(const ros::NodeHandle& nh);

std::unique_ptr<x_view::Parameters> parseMatcher(const ros::NodeHandle& nh);
std::unique_ptr<x_view::Parameters> parseGraphMatcher(const ros::NodeHandle& nh);
std::unique_ptr<x_view::Parameters> parseVectorMatcher(const ros::NodeHandle& nh);

}

#endif //X_VIEW_PARSER_H
