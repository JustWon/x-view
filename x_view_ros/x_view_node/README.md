# X-View Node
This repository contains the implementation of the __X-View Node__ 
package. This package implements a rosnode which registers itself to the 
topics passed as _rosparams_ listed below:
  * `/XViewWorker/semantics_image_topic` Specifies the rostopic over which 
  the semantic images are sent.
  * `/XViewWorker/depth_image_topic` Specifies the rostopic over which the 
  depth images are sent.
