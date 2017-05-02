#include <gtest/gtest.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/mcgregor_common_subgraphs.hpp>

#include <x_view_core/x_view_types.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/datasets/synthia_dataset.h>

using namespace x_view;

// create a callback object called for each found subgraph

template<typename G>
struct print_callback {

  print_callback(const G& graph1, const G& graph2) :
      m_graph1(graph1), m_graph2(graph2) {}

  template<typename CorrespondenceMapFirstToSecond,
      typename CorrespondenceMapSecondToFirst>
  bool operator()(CorrespondenceMapFirstToSecond correspondence_map_1_to_2,
                  CorrespondenceMapSecondToFirst correspondence_map_2_to_1,
                  typename boost::graph_traits<G>::vertices_size_type
                  subgraph_size) {

    auto v_begin = boost::vertices(m_graph1).first;
    auto v_end = boost::vertices(m_graph1).second;
    for (; v_begin != v_end; ++v_begin) {
      auto v = *v_begin;
      // Skip unmapped vertices
      auto correspondence = boost::get(correspondence_map_1_to_2, v);
      if (correspondence != boost::graph_traits<G>::null_vertex()) {
        std::cout << "vertex: \t" << v << ", " << m_graph1[v]
                  << " \t <-> \t "
                  << "vertex: \t" << correspondence << ", "
                  << m_graph2[correspondence]
                  << std::endl;
      }

    }

    std::cout << "---" << std::endl;
  }

 private:
  const G& m_graph1;
  const G& m_graph2;

};

struct VertexData {

  VertexData() : label_(0), label_name_(global_dataset_ptr->label(0)) {}
  VertexData(const int label)
      : label_(label),
        label_name_(global_dataset_ptr->label(label_)) {}
  int label_;
  std::string label_name_;

};

std::ostream& operator<<(std::ostream& out, const VertexData& v) {
  return out << "label: \t" << std::to_string(v.label_)
             << ", name: \t" << v.label_name_;
}

TEST(XViewSlamTestSuite, graphMatching

) {

// initialize a dataset
global_dataset_ptr = std::make_shared<SynthiaDataset>();

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              VertexData> GraphType;

typedef GraphType::vertex_descriptor VertexDescriptor;

typedef GraphType::edge_descriptor EdgeDescriptor;

// Build first graph (circle)
GraphType g0;

auto v0_0 = boost::add_vertex({0}, g0);

auto v0_1 = boost::add_vertex({1}, g0);

auto v0_2 = boost::add_vertex({2}, g0);

auto v0_3 = boost::add_vertex({3}, g0);

auto v0_4 = boost::add_vertex({4}, g0);

auto v0_5 = boost::add_vertex({5}, g0);

boost::add_edge(v0_0, v0_1, g0

);

boost::add_edge(v0_1, v0_2, g0

);

boost::add_edge(v0_2, v0_3, g0

);

boost::add_edge(v0_3, v0_4, g0

);

boost::add_edge(v0_4, v0_5, g0

);

boost::add_edge(v0_5, v0_0, g0

);

// Build second graph partially overlapping the previous one
GraphType g1;

auto v1_0 = boost::add_vertex({2}, g1);

auto v1_1 = boost::add_vertex({3}, g1);

auto v1_2 = boost::add_vertex({10}, g1);

auto v1_3 = boost::add_vertex({5}, g1);

auto v1_4 = boost::add_vertex({0}, g1);

boost::add_edge(v1_0, v1_1, g1

);

boost::add_edge(v1_1, v1_2, g1

);

boost::add_edge(v1_2, v1_0, g1

);

boost::add_edge(v1_2, v1_3, g1

);

boost::add_edge(v1_3, v1_4, g1

);

// find maximal common subgraph
auto property_map_0 = boost::get(&VertexData::label_, g0);

auto property_map_1 = boost::get(&VertexData::label_, g1);

// we want to find subgraph with the following comparison between nodes
auto are_nodes_the_same = boost::vertices_equivalent
    (boost::make_property_map_equivalent(property_map_0, property_map_1));

print_callback<GraphType> my_callback(g0, g1);

boost::mcgregor_common_subgraphs_maximum_unique(g0, g1,

true, my_callback,
are_nodes_the_same);



// find maximally common subgraph

}

