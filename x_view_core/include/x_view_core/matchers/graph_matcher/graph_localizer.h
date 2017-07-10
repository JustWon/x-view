#ifndef X_VIEW_GRAPH_LOCALIZER_H
#define X_VIEW_GRAPH_LOCALIZER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/x_view_types.h>

namespace x_view {

class GraphLocalizer {

 public:
  GraphLocalizer();

  GraphLocalizer &operator=(GraphLocalizer const& f) = delete;

  void addObservation(const VertexProperty& vertex_property,
                      const Eigen::Vector3d observation);

  const Eigen::Vector3d localize() const;

 private:
  struct Observation {
    VertexProperty vertex_property;
    Eigen::Vector3d measurement;
  };

  std::vector<Observation> observations_;
};

}

#endif //X_VIEW_GRAPH_LOCALIZER_H
