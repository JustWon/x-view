#include <thread>

#include <ros/ros.h>

#include <x-view_node/x-view_worker.h>

int main(int argc, char **argv) {

    // name cannot contain '-' char
    ros::init(argc, argv, "XView");
    ros::NodeHandle node_handle("~");

    x_view_ros::XViewWorker worker(node_handle);

    try {
        ros::spin();
    }
    catch (const std::exception &e) {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return 1;
    }
    catch (...) {
        ROS_ERROR_STREAM("Unknown Exception");
        return 1;
    }

    return 0;
}
