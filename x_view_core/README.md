# X-View core
This repository contains the core implementation of the X-View library

## Setting up XView

### Prerequisites

First install the required system packages:
```sh
$ sudo apt-get install python-wstool doxygen
```

### Building
Clone the XView repository in the local `src` folder:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ethz-asl/x-view.git
```

Download the dependencies of XView using `wstool`:
```sh
$ wstool init
$ wstool merge x-view/x_view_core/dependencies.rosinstall
$ wstool update
```

Build the entire project (this might take up to 30 minutes)
```sh
$ cd ~/catkin_ws
$ catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin build --force-cmake -j8
```

Build and run XView tests
```sh
$ catkin build --force-cmake -j8 --catkin-make-args tests
$ ./devel/libs/x_view_core/x_view_core_tests
```
