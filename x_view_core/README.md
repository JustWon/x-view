# X-View core
This repository contains the core implementation of the X-View library

## Setting up XView

### Prerequisites

First install the required system packages:
```sh
$ sudo apt-get install python-wstool doxygen
```

### Dependencies
We use wstool for installing the catkin dependencies, i.e.,
```sh
$ cd ~/catkin_ws/src
$ wstool init
$ wstool merge x-view/x_view_core/dependencies.rosinstall
$ wstool update
```

### Building
The following steps build XView using _catkin_ assuming all the _dependencies_ listed above and _XView_ are already downloaded in the _~/git_ folder of your machine
```sh
# create a new catkin workspace
$ mkdir -p ~/x_view_ws/src && cd x_view_ws
# initialize the workspace (either in 'Release' or 'Debug' mode)
$ catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
# create symbolic link to the source
$ for dir in ~/git/*/; do ln -s ${dir} src/; done
# build all packages
$ catkin build --force-cmake -j8 
```
**Note**: building the entire project takes around 30 minutes.

### Tests
All test are implemented as _gtest_ in the [test](./test) folder. The tests can be launched as follows:
```sh
# build the tests
$ catkin build --force-cmake --catkin-make-args tests
# run the tests through catkin
$ catkin build --catkin-make-args run_tests
# or by running the executable
$ ./devel/lib/x_view_core/x_view_core_tests 
```
