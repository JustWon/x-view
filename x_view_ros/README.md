# X-View ros
This repository contains the implementation of the ros nodes and ros 
utilities used to run the __X-View__ library as a ros node.

### Structure
Here follows a description of the folder structure adopted in the 
development of this project.

* [__x_view_bag_reader__](./x_view_bag_reader) directory containing the 
implementation of a ros node which actively parses a rosbag and calls 
X-View functions on data extracted by it.

* [__x_view_graph_publisher__](./x_view_graph_publisher) directory containing
the implementation of a _graph publisher_ class, which allows a ros node to 
publish a semantic graph as a ros message. This message can be read by tools 
such as RViz for visualization purposes.

* [__x_view_node__](./x_view_node) directory containing the implementation of
 a ros node which passively listens to the topics published by playing a 
 rosbag and submits the data to X-View. Note that this node implementation 
 does not allow the used to have a direct control on which data is set to 
 X-View, but rather behaves like a bridge between the data contained in the 
 rosbag and X-View.

* [__x_view_parser__](./x_view_parser) directory containing the 
implementation of a _parser_ structure, which parses parameters of interest 
from the ros param server and stores them in `x_view::Locator::parameters_`, 
making the parameters accessible from X-View without needing to access the 
ros param server.


### Running ros nodes
In order to run the ros nodes provided by this repository, the X-View project
 must be compiled.

After compilation (see [here](../x_view_core/README.md#Building)) and 
resourcing the current catkin directory (`$ source ./devel/setup.bash`) the 
different ros nodes can be run as follows:
* __x_view_bag_reader__:
   ```lang:bash
   $ roslaunch x_view_bag_reader x_view_bag_node.launch 
   ```
   
* __x_view_node__:
   ```lang:bash
   $ roslaunch x_view_node x_view_simulator.launch
   ```
   
### X-View parameters
The initial parameters used by X-View are the ones store in the corresponding 
`cfg` directory inside the ros node directory (for _x_view_bag_reader_ see 
[here](./x_view_bag_reader/cfg), for _x_view_node_ see
[here](./x_view_node/cfg)).
 
Here follows a description of the parameters being used by X-View:
 
* __Dataset__:
   * __name__: string referring to the name of the dataset being used.<br/>
   _supported values_: {"SYNTHIA", "AIRSIM"} 
  
* __Landmark__
   * __type__ : string referring the type of landmark to be used. <br/>
   _supported values_: {"GRAPH", "ORB", "SIFT", "SURF", "HISTOGRAM"}<br/></br>
   
   * __blob_filter_type__(type="GRAPH"): string referring to the the filter 
   type 
   to be used by X-View when extracting blobs from the semantically segmented
   input image.<br/>
   _supported values_: {"ABSOLUTE", "RELATIVE"}
    
   * __min_blob_size__(type="GRAPH" & blob_filter_type="ABSOLUTE"): integer 
   representing the 
   minimum 
   number of pixels a blob must have to be considered as valid. All blobs 
   containing a smaller number of pixels are simply ignored.
   
   * __min_blob_size__(type="GRAPH" & blob_filter_type="RELATIVE"): floating 
   point value representing 
   the surface fraction of the input image a blob must cover to be 
   considered as valid. All blobs whose surface is smaller than the passed 
   fraction are simply ignored.
      
   * __dilate_and_erode__(type="GRAPH"): boolean indicating whether a _dilation_ 
   and _erosion_ step has to be done on the blobs when they get extracted 
   from the image. Enabling this procedure allows to _"fill"_ small holes in 
   blobs and smooths their border.
   
   * __num_dilate__(type="GRAPH" & dilate_and_erode"true") 
   integer indicating how many times the _dilation_ procedure has to be applied to the extracted blob.
   The larger this number, the larger the blob gets and contour details are 
   lost.
   
   * __num_erode__(type="GRAPH" & dilate_and_erode="true") integer indicating 
   how many times the _erosion_ procedure has to be applied to the extracted blob.
   This number is usually the same as __num_dilate__.
   
   * __blob_neighbor_distance__(type="GRAPH") integer indicating how many 
   pixels far apart two blobs can be to still consider them as neighbors.
   
   * __depth_clip__(type="GRAPH") floating point number indicating the 
   maximal depth value associated to a blob in order to be considered. All 
   blobs whose depth value is larger than this parameter are simply ignored.
  
   * __num_features__(type="ORB" | type="SIFT" | type="SURF") integer 
   indicating how many visual features must be extracted from the input image.
   
   * __hessian_threshold__(type="SURF") floating point number indicating the 
   _hessian threshold_ to be used when extracting SURF features.
  

* __Matcher__
   * __type__: string referring the type of matcher to be used. <br/>
     _supported values_: {"GRAPH", "VECTOR"}<br/></br>
   
   * __vertex_similarity_score__(type="GRAPH") string representing the 
   similarity type to be used when computing the pairwise similarity between 
   vertices of a semantic graph. <br/>
   _supported values_: {"WEIGHTED", "SURFACE"}
   
   * __random_walk_sampling_type__(type="GRAPH") string representing the 
   random walk type to be adopted when computing the vertex descriptors.<br/>
   _supported values_: {"UNIFORM", "AVOIDING", "WEIGHTED"}
   
   * __num_walks__(type="GRAPH") integer indicating how many random walks to 
   compute for each vertex.
   
   * __walk_length__(type="GRAPH") integer indicating the length of the random
    walks computed for each vertex of the semantic graph. This number 
    indicates the number of _steps_ of the random walk, thus if this 
    parameter is equal to _N_, then the random walk will have _N+1_ vertices.
   
   * __time_window__(type="GRAPH") integer indicating the time window to be 
   used during graph matching, i.e. the maximal time-distance between two 
   semantic vertices to be valid matches. If this parameter is set to zero, 
   then no time restriction is applied to the match.
   
   * __similarity_threshold__(type="GRAPH") floating point value indicating 
   the minimal similarity between two candidate matching vertices to be 
   considered as valid. Candidate matches whose semantic similarity score is 
   smaller than this parameter are rejected.
   
   * __distance_threshold__(type=="GRAPH") floating point value indicating 
   the maximal Euclidean distance between the vertices of a candidate match 
   to be considered as valid. If the vertices representing a candidate match 
   are further apart than this parameter, the match is rejected.
   
   * __merge_distance__(type="GRAPH") floating point value indicating the 
   maximal Euclidean distance between two vertices in order to be merged 
   together. Given a pair of vertices with same semantic label and whose 
   distance is smaller than this parameter will make them merge together.
   
   * __num_retained_matches__(type="VECTOR") integer indicating how many 
   matches have to be retained for each feature. If this paremeter is equal 
   to one, then only the best match is retained.
  
### X-View-Worker parameters
  
* __XViewWorker__
   * __semantics_image_topic__ string representing the topic to which the 
   XViewWorker must listen to for semantic images.<br/>
   _example value_: "/Stereo_Left/Omni_{F/R/B}/labels"
   
   * __depth_image_topic__ string representing the topic to which the 
   XViewWorker must listen to for depth images.<br/>
   _example value_: "/Stereo_Left/Omni_{F/R/B}/depth"
   
   * __sensor_frame__ string representing the ID of the sensor frame.</br>
   _example value_: "cam0{0/1/2}"
   
   * __world_frame__ string representing the ID of the world frame.</br>
   _example value_: "world"
  }
  
  
  
  
### X-View-Bag-Reader parameters

* __XViewBagReader__
   * __bag_file_name__ string representing the absolute path to the rosbag 
   file to be parsed.</br>
   _example value_: "/home/carlo/polybox/Master/Master 
   thesis/x-view/x_view_ros/x_view_node/data/Synthia_3_cams.bag"
   
   * __front__
       * __semantics_image_topic__ string representing the topic associated to
        the semantic images of the __front__ camera.<br/>
        _example value_: "/Stereo_Left/Omni_F/labels"
          
       * __depth_image_topic__ string representing the topic associated to 
       the depth images of the __front__ camera.<br/>
       _example value_: "/Stereo_Left/Omni_F/depth"
         
       * __sensor_frame__: string representing the ID of the sensor frame 
       of the __front__ camera.<br/>
       _example value_: "cam00"
         
   * __right__
       * __semantics_image_topic__ string representing the topic associated to
       the semantic images of the __right__ camera.<br/>
       _example value_: "/Stereo_Left/Omni_R/labels"
          
       * __ depth_image_topic__ string representing the topic associated to 
       the depth images of the __right__ camera.<br/>
       _example value_: "/Stereo_Left/Omni_B/depth"
         
       * __sensor_frame__: string representing the ID of the sensor frame 
       of the __right__ camera.<br/>
       _example value_: "cam01"
   
   * __back__
      * __semantics_image_topic__ string representing the topic associated to
       the semantic images of the __back__ camera.<br/>
       _example value_: "/Stereo_Left/Omni_B/labels"
       
      *__depth_image_topic__ string representing the topic associated to the 
      depth images of the __back__ camera.<br/>
      _example value_: "/Stereo_Left/Omni_B/depth"
      
      * __sensor_frame__: string representing the ID of the sensor frame 
      of the __back__ camera.<br/>
      _example value_: "cam02"
      
      
  
   * __transform_topic__ string representing the topic associated to the 
   transform tree.<br/>
   _example value_: "/tf"
   
   * __world_frame__: string representing the ID of the world frame.</br>
     _example value_: "world"
  }

