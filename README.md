# X-View
This repository contains projects related to X-View.

### Developement
* [x_view_core](./x_view_core) contains the implementation of semantic SLAM.
* [x_view_ros](./x_view_ros) uses _x_view_core_ as a ros package. It provides
 different utilities and _ros nodes_ to execute [X-View](./x_view_core).
* [topograph-matching](./topograph-matching) computes semantic similarities between nodes contained in a graph structure and applies virtual forces to the graph-nodes in order to match them
* [semantic-graph-matching](./semantic-graph-matching) Simulates a small 2D planar grid-style word where two robots explore the map and observe semantic landmarks. Those landmarks are used to perform node-matching on a semantic level by performing a voting scheme

### External libraries testing
* [wordnet-examples](./wordnet-examples) shows how one can query semantic similarity measures on the wordnet database.
* [g2o-examples](./g2o-examples) contains some tests exploring the g2o library for graph optimization
* [gtsam-examples](./gtsam-examples) constains some tests exploring the GTSAM library for graph optimization


