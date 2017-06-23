#ifndef X_VIEW_PARSER_H
#define X_VIEW_PARSER_H

#include <x_view_core/parameters/parameters.h>

#include <ros/ros.h>

#include <memory>

namespace x_view_ros {

class Parser {
 public:

  Parser(const ros::NodeHandle& nh)
      : nh_(nh) {}

  virtual std::unique_ptr<x_view::Parameters> parseParameters() const;

 protected:
  const ros::NodeHandle& nh_;

  static void addString(const ros::NodeHandle& nh, const std::string& ros_key,
                        const std::string& param_key,
                        std::unique_ptr<x_view::Parameters>& parameters,
                        const std::string& default_string = "");

  static void addInt(const ros::NodeHandle& nh, const std::string& ros_key,
                     const std::string& param_key,
                     std::unique_ptr<x_view::Parameters>& parameters,
                     const int default_int = 0);

  static void addFloat(const ros::NodeHandle& nh, const std::string& ros_key,
                       const std::string& param_key,
                       std::unique_ptr<x_view::Parameters>& parameters,
                       const float default_float = 0.f);

  static void addBool(const ros::NodeHandle& nh, const std::string& ros_key,
                      const std::string& param_key,
                      std::unique_ptr<x_view::Parameters>& parameters,
                      const bool default_bool = false);

  std::unique_ptr<x_view::Parameters> parseDataset() const;
  std::unique_ptr<x_view::Parameters> parseLandmark() const;
  std::unique_ptr<x_view::Parameters> parseMatcher() const;

 private:

  std::unique_ptr<x_view::Parameters> parseHistogramLandmark() const;
  std::unique_ptr<x_view::Parameters> parseORBLandmark() const;
  std::unique_ptr<x_view::Parameters> parseSIFTLandmark() const;
  std::unique_ptr<x_view::Parameters> parseSURFLandmark() const;
  std::unique_ptr<x_view::Parameters> parseGraphLandmark() const;

  std::unique_ptr<x_view::Parameters> parseGraphMatcher() const;
  std::unique_ptr<x_view::Parameters> parseVectorMatcher() const;

};

}

#endif //X_VIEW_PARSER_H
