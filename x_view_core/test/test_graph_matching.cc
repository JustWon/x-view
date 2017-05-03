#include <gtest/gtest.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/mcgregor_common_subgraphs.hpp>

#include <x_view_core/x_view_types.h>

using namespace x_view;

// global variable that counts edges in maximal common subgraph. It is ugly
// to have it global, but it is not possible to store as a member variable of
// SubgraphCallback as the callback object is continuously destroyed and
// restored by the mcgregor function.
int edge_count_;

/// \brief Object representing a vertex of the graph.
struct VertexData {

  VertexData() : label_("") {}
  VertexData(const std::string& label)
      : label_(label) {}
  std::string label_;
};

std::ostream& operator<<(std::ostream& out, const VertexData& v) {
  return out << v.label_;
}

/// \brief a Graph is represented as a boost adjacency list.
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              VertexData> GraphType;

typedef GraphType::vertex_descriptor VertexDescriptor;

/**
 * \brief Each time a subgraph is found by the mcgregor algorithm, the
 * operator () of this object is called.
 * \tparam G Graph type.
 */
template<typename G>
struct SubgraphCallback {

  SubgraphCallback(const G& graph1, const G& graph2) :
      graph1_(graph1), graph2_(graph2) {}

  template<typename CorrespondenceMapFirstToSecond,
      typename CorrespondenceMapSecondToFirst>
  bool operator()(CorrespondenceMapFirstToSecond correspondence_map_1_to_2,
                  CorrespondenceMapSecondToFirst correspondence_map_2_to_1,
                  typename boost::graph_traits<G>::vertices_size_type
                  subgraph_size) {

    auto v_begin = boost::vertices(graph1_).first;
    auto v_end = boost::vertices(graph1_).second;
    for (; v_begin != v_end; ++v_begin) {
      auto v = *v_begin;
      // Skip unmapped vertices
      auto correspondence = boost::get(correspondence_map_1_to_2, v);
      if (correspondence != boost::graph_traits<G>::null_vertex()) {
        std::cout << "vertex: \t" << v << ", " << graph1_[v]
                  << " \t <-> \t "
                  << "vertex: \t" << correspondence << ", "
                  << graph2_[correspondence]
                  << std::endl;
      }

    }

    std::cout << "-------" << std::endl;

    int current_subgraph_edges = 0;
    for (auto v1_begin = boost::vertices(graph1_).first; v1_begin != v_end;
         ++v1_begin) {
      auto v1 = *v1_begin;
      auto correspondence1 = boost::get(correspondence_map_1_to_2, v1);
      if (correspondence1 != boost::graph_traits<G>::null_vertex())
        for (auto v2_begin = v1_begin + 1; v2_begin != v_end; ++v2_begin) {
          auto v2 = *v2_begin;
          auto correspondence2 = boost::get(correspondence_map_1_to_2, v2);
          if (correspondence2 != boost::graph_traits<G>::null_vertex()) {
            if (boost::edge(v1, v2, graph1_).second) {
              ++current_subgraph_edges;
            }
          }
        }
    }

    edge_count_ = std::max(edge_count_, current_subgraph_edges);
  }

 private:
  const G& graph1_;
  const G& graph2_;

};

TEST(XViewSlamTestSuite, graphMatching) {


  // Build up graphs as defined in Fig 3 of
  // https://openproceedings.org/2012/conf/edbt/ZhuQYC12.pdf

  // Build first graph (q)
  GraphType q;

  VertexDescriptor v0_0 = boost::add_vertex({"C"}, q);
  VertexDescriptor v0_1 = boost::add_vertex({"C"}, q);
  VertexDescriptor v0_2 = boost::add_vertex({"B"}, q);
  VertexDescriptor v0_3 = boost::add_vertex({"A"}, q);
  VertexDescriptor v0_4 = boost::add_vertex({"B"}, q);
  VertexDescriptor v0_5 = boost::add_vertex({"A"}, q);
  VertexDescriptor v0_6 = boost::add_vertex({"C"}, q);
  VertexDescriptor v0_7 = boost::add_vertex({"B"}, q);
  VertexDescriptor v0_8 = boost::add_vertex({"C"}, q);
  VertexDescriptor v0_9 = boost::add_vertex({"C"}, q);
  VertexDescriptor v0_10 = boost::add_vertex({"B"}, q);

  boost::add_edge(v0_0, v0_1, q);
  boost::add_edge(v0_1, v0_2, q);
  boost::add_edge(v0_0, v0_2, q);
  boost::add_edge(v0_2, v0_3, q);
  boost::add_edge(v0_3, v0_4, q);
  boost::add_edge(v0_3, v0_5, q);
  boost::add_edge(v0_5, v0_6, q);
  boost::add_edge(v0_6, v0_7, q);
  boost::add_edge(v0_7, v0_8, q);
  boost::add_edge(v0_8, v0_9, q);
  boost::add_edge(v0_9, v0_10, q);
  boost::add_edge(v0_6, v0_10, q);



  // Build second graph (g1)
  GraphType g1;
  VertexDescriptor v1_0 = boost::add_vertex({"C"}, g1);
  VertexDescriptor v1_1 = boost::add_vertex({"C"}, g1);
  VertexDescriptor v1_2 = boost::add_vertex({"B"}, g1);
  VertexDescriptor v1_3 = boost::add_vertex({"A"}, g1);
  VertexDescriptor v1_4 = boost::add_vertex({"B"}, g1);
  VertexDescriptor v1_5 = boost::add_vertex({"A"}, g1);

  boost::add_edge(v1_0, v1_1, g1);
  boost::add_edge(v1_1, v1_2, g1);
  boost::add_edge(v1_0, v1_2, g1);
  boost::add_edge(v1_2, v1_3, g1);
  boost::add_edge(v1_3, v1_4, g1);
  boost::add_edge(v1_3, v1_5, g1);

  // Build third graph (g2)
  GraphType g2;
  VertexDescriptor v2_0 = boost::add_vertex({"C"}, g2);
  VertexDescriptor v2_1 = boost::add_vertex({"C"}, g2);
  VertexDescriptor v2_2 = boost::add_vertex({"B"}, g2);
  VertexDescriptor v2_3 = boost::add_vertex({"A"}, g2);
  VertexDescriptor v2_4 = boost::add_vertex({"B"}, g2);
  VertexDescriptor v2_5 = boost::add_vertex({"C"}, g2);
  VertexDescriptor v2_6 = boost::add_vertex({"C"}, g2);
  VertexDescriptor v2_7 = boost::add_vertex({"B"}, g2);
  VertexDescriptor v2_8 = boost::add_vertex({"C"}, g2);
  VertexDescriptor v2_9 = boost::add_vertex({"C"}, g2);
  VertexDescriptor v2_10 = boost::add_vertex({"B"}, g2);

  boost::add_edge(v2_0, v2_1, g2);
  boost::add_edge(v2_1, v2_2, g2);
  boost::add_edge(v2_0, v2_2, g2);
  boost::add_edge(v2_2, v2_3, g2);
  boost::add_edge(v2_3, v2_4, g2);
  boost::add_edge(v2_3, v2_5, g2);
  boost::add_edge(v2_5, v2_6, g2);
  boost::add_edge(v2_6, v2_7, g2);
  boost::add_edge(v2_7, v2_8, g2);
  boost::add_edge(v2_8, v2_9, g2);
  boost::add_edge(v2_9, v2_10, g2);
  boost::add_edge(v2_6, v2_10, g2);

  auto mcs_distance = [](const GraphType& a, const GraphType& b) -> int {
    // find maximal common subgraph
    auto property_map_a = boost::get(&VertexData::label_, a);
    auto property_map_b = boost::get(&VertexData::label_, b);

    // we want to find subgraph with the following comparison between nodes
    auto are_nodes_the_same = boost::vertices_equivalent
        (boost::make_property_map_equivalent(property_map_a, property_map_b));

    SubgraphCallback<GraphType> my_callback(a, b);
    bool only_connected_subgraphs = false;
    edge_count_ = 0;
    boost::mcgregor_common_subgraphs_maximum_unique(a, b,
                                                    only_connected_subgraphs,
                                                    my_callback,
                                                    are_nodes_the_same);

    const int num_edges_in_subgraph = edge_count_;

    const int num_edges_a = boost::num_edges(a);
    const int num_edges_b = boost::num_edges(b);

    const int distance = num_edges_a + num_edges_b - 2 * num_edges_in_subgraph;

    return distance;
  };

  std::cout << "Measuring common subgraph between q and g1" << std::endl;
  std::cout << "--> distance: " << mcs_distance(q, g1) << std::endl;

  std::cout << "Measuring common subgraph between q and g2" << std::endl;
  std::cout << "--> distance: " << mcs_distance(q, g2) << std::endl;

  std::cout << "Measuring common subgraph between q and q" << std::endl;
  std::cout << "--> distance: " << mcs_distance(q, q) << std::endl;

}

