# X-View core
This repository contains the core implementation of the X-View library

### Tests
All test are implemented as _gtest_ in the [test](./test) folder. The tests can be launched as follows:
```lang=bash
catkin build --force-cmake
./devel/libs/x_view_core/x_view_core_tests
```

### Notes
* Since the library relies on _opencv_, and since the distribution provided by _ros_indigo_ does not provide the 'nonfree' module
containing useful implementation of vision algorithms, one should download it by following the intructions reported [here](http://gabrieleomodeo.it/blog/how-to-set-up-ros-with-opencv2-nonfree-package/)
