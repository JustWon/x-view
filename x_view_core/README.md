# X-View core
This repository contains the core implementation of the X-View library

### Tests
All test are implemented as _gtest_ in the [test](./test) folder. The tests can be launched as follows:
```lang=bash
catkin build --force-cmake --catkin-make-args run_tests
./devel/lib/x_view_core/x_view_core_tests
```

**Note**: building the entire project takes around 30 minutes.

### Build type
Since some operations performed by XView are computational intensive (image processing routines etc), in order to speed up the program execution one could want to build the entire project in _RELEASE_ mode. To do so, these are the commands to be executed:
```lang=bash
# create 'build' and 'devel' folders for release compilation
catkin config --profile release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release
# compile the 'release' profile making sure to compile and run the tests
catkin build --profile release --force-cmake --catkin-make-args run_tests
```

The tests compiled in release mode can be executed as follows:
```lang=bash
./devel_release/lib/x_view_core/x_view_core_tests
```
### Notes
* Since the library relies on _opencv_, and since the distribution provided by _ros_indigo_ does not provide the 'nonfree' module
containing useful implementation of vision algorithms, one should download it by following the intructions reported [here](http://gabrieleomodeo.it/blog/how-to-set-up-ros-with-opencv2-nonfree-package/)
