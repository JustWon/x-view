#ifndef X_VIEW_GRAPH_PUBLISHER_H
#define X_VIEW_GRAPH_PUBLISHER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace x_view_ros {

/**
 * \brief This class can be used as a wrapper for publishing data stored in a
 * x_view::Graph object.
 */
class GraphPublisher {
 public:
  /**
   * \brief Constructor of GraphPublisher object.
   * \param nh Node handle reference used for publishing ros messages.
   * \param vertex_topic Topic to which vertex coordinates are published.
   * \param edge_topic Topic to which edges are published.
   */
  GraphPublisher(ros::NodeHandle& nh,
                 const std::string& vertex_topic = "/graph/vertices",
                 const std::string& edge_topic = "/graph/edges",
                 const std::string& matches_topic = "/graph/matches");

  /**
   * \brief Publishes all information contained in the graph passed as argument.
   * \param graph Graph object to be published.
   * \param time Time at which the graph is published.
   * \param z_offset Offset in z, in which the graph is to be published.
   */
  void publish(const x_view::Graph& graph, const ros::Time& time,
               double z_offset = 0.0) const;

  /**
    * \brief Publishes all matches between query and database graph.
    * \param query_graph Local Graph object.
    * \param database_graph Global Graph object.
    * \param candidate_matches Matches between query and database graph.
    * \param time Time at which the matches are published.
    * \param z_offset Offset in z, in which the graph is to be published.
    */
  void publishMatches(
      const x_view::Graph& query_graph, const x_view::Graph& database_graph,
      const x_view::GraphMatcher::IndexMatrixType& candidate_matches,
      const ros::Time& time, double z_offset = 0.0) const;

 private:
  /// \brief Reference to the ros node handle used for publishing messages.
  ros::NodeHandle& nh_;
  /// \brief Topic to which vertex coordinates are published.
  const std::string vertex_topic_;
  /// \brief Topic to which edges are published.
  const std::string edge_topic_;
  /// \brief Topic to which matches are published.
  const std::string matches_topic_;

  /// \brief Vertex publisher object used for publishing vertex coordinates.
  ros::Publisher vertex_publisher_;
  /// \brief Edges publisher object used for publishing edges.
  ros::Publisher edge_publisher_;
  /// \brief Matches publisher object used for publishing matches.
  ros::Publisher matches_publisher_;

  /// \brief Routine used internally to publish all vertices of the graph
  /// passed as argument.
  void publishVertices(const x_view::Graph& graph, const ros::Time& time,
                       double z_offset = 0.0) const;

  /// \brief Routine used internally to publish all edges of the graph passed
  /// as argument.
  void publishEdges(const x_view::Graph& graph, const ros::Time& time,
                    double z_offset = 0.0) const;



};
}

#endif //X_VIEW_GRAPH_PUBLISHER_H
