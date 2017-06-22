#ifndef X_VIEW_TEST_PARAM_TREE_H
#define X_VIEW_TEST_PARAM_TREE_H

#include <glog/logging.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_map>


namespace x_view_test {

void testParameterValues();

void testParameterChildren();

/*
struct Parameters {
  Parameters() : base_name("/") {}
  std::string base_name;
  virtual void load(const ros::NodeHandle& nh) = 0;
  virtual const std::string toString() const  = 0;
};

struct LandmarkParameters : public Parameters {
  LandmarkParameters()
      : Parameters() {
    base_name = "/XView/landmarks/";
  }

  std::string type;

  virtual const std::string toString() const = 0;
};


struct GraphLandmrkParameters : public LandmarkParameters {
  GraphLandmrkParameters()
      : LandmarkParameters() {
    type = "GRAPH";
  }

  int min_blob_size;

  virtual void load(const ros::NodeHandle& nh) {
    nh.getParam(base_name + "min_blob_size", min_blob_size);
  }

  virtual const std::string toString() const {
    std::string s;
    s += "type: " + type + ", ";
    s += "min_blob_size: " + std::to_string(min_blob_size);
    return s;
  }
};

struct MatcherParameters : public Parameters {
  MatcherParameters()
      : Parameters() {
    base_name = "/XView/matcher/";
  }

  std::string type;

  virtual const std::string toString() const = 0;
};

struct GraphMatcherParameters : public MatcherParameters {
  GraphMatcherParameters()
      : MatcherParameters() {
    type = "GRAPH";
  }

  int walk_length;

  virtual void load(const ros::NodeHandle& nh) {
    nh.getParam(base_name + "walk_length", walk_length);
  }

  virtual const std::string toString() const {
    std::string s;
    s += "type: " + type + ", ";
    s += "walk_length: " + std::to_string(walk_length);
    return s;
  }
};

struct XViewParameters : public Parameters {
  XViewParameters()
      : Parameters() {
    base_name = "/XView/";
  }

  MatcherParameters* matcher_parameters;
  LandmarkParameters* landmark_parameters;

  virtual void load(const ros::NodeHandle& nh) {
    std::string matcher_type;
    nh.getParam(base_name + "matcher/type", matcher_type);
    if(matcher_type == "GRAPH")
      matcher_parameters = new GraphMatcherParameters();
    else
      std::cout << "Unrecognized matcher type <"<<matcher_type << ">"
          << std::endl;

    matcher_parameters->load(nh);

    std::string landmark_type;
    nh.getParam(base_name + "landmarks/type", landmark_type);
    if(landmark_type == "GRAPH")
      landmark_parameters = new GraphLandmrkParameters();
    else
      std::cout << "Unrecognized landmark type <"<<landmark_type << ">"
                << std::endl;

    landmark_parameters->load(nh);

  }

  virtual const std::string toString() const {
    std::string s;
    s += "Matcher: " + matcher_parameters->toString() + ", ";
    s += "\nLandmark: " + landmark_parameters->toString();
    return s;
  }
};

struct XViewBagReaderParameters : public Parameters {
  XViewBagReaderParameters()
      : Parameters() {
    base_name = "/XViewBagReader/";
  }
  std::string dataset;
  std::string sensor_frame;
  std::string world_frame;

  virtual void load(const ros::NodeHandle& nh) {
    nh.getParam(base_name + "dataset", dataset);
    nh.getParam(base_name + "sensor_frame", sensor_frame);
    nh.getParam(base_name + "world_frame", world_frame);
  }

  virtual const std::string toString() const {
    std::string s;
    s += "Dataset: " + dataset + ", ";
    s += "\nSensor frame: " + sensor_frame + ", ";
    s += "\nWorld frame: " + world_frame;
    return s;
  }

};

struct GlobalParameters : public Parameters {
  GlobalParameters()
      : x_view_parameters(new XViewParameters()),
        bag_reader_parameters(new XViewBagReaderParameters())
  {}

  XViewParameters* x_view_parameters;
  XViewBagReaderParameters* bag_reader_parameters;

  virtual void load(const ros::NodeHandle& nh) {
    x_view_parameters->load(nh);
    bag_reader_parameters->load(nh);
  }

  virtual const std::string toString() const {
    std::string s;
    s += "XView: " + x_view_parameters->toString() + ", ";
    s += "\nBag reader: " + bag_reader_parameters->toString();
    return s;
  }

};

 */

}

#endif //X_VIEW_TEST_PARAM_TREE_H
