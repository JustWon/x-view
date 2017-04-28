# X-View core
This repository contains the core implementation of the X-View library

## Setting up XView

### Dependencies
XView is structured as a _catkin_ package, and has the following dependencies (available on [github](https://github.com/)):
* [catkin_simple](https://github.com/catkin/catkin_simple)
* [doxygen_catkin](https://github.com/ethz-asl/doxygen_catkin)
* [eigen_catkin](https://github.com/ethz-asl/eigen_catkin)
* [eigen_checks](https://github.com/ethz-asl/eigen_checks)
* [gflags_catkin](https://github.com/ethz-asl/gflags_catkin)
* [glog_catkin](https://github.com/ethz-asl/glog_catkin)
* [gtsam_catkin](https://github.com/ethz-asl/gtsam_catkin)
* [minkindr](https://github.com/ethz-asl/minkindr)
* [minkindr_ros](https://github.com/ethz-asl/minkindr_ros) 
* [opencv2_catkin](https://github.com/ethz-asl/opencv2_catkin)

### Building
The following steps build XView using _catkin_ assuming all the _dependencies_ listed above and _XView_ are already downloaded in the _~/git_ folder of your machine
```lang=bash
# create a new catkin workspace
mkdir -p ~/x_view_ws/src && cd x_view_ws
# initialize the workspace (either in 'Release' or 'Debug' mode)
catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
# create symbolic link to the source
for dir in ~/git/*/; do ln -s ${dir} src/; done
# build all packages
catkin build --force-cmake -j8 
```
**Note**: building the entire project takes around 30 minutes.

### Tests
All test are implemented as _gtest_ in the [test](./test) folder. The tests can be launched as follows:
```lang=bash
# build the tests
catkin build --force-cmake --catkin-make-args tests
# run the tests through catkin
catkin build --catkin-make-args run_tests
# or by running the executable
./devel/lib/x_view_core/x_view_core_tests 
```
