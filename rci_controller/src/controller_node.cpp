#include <ros/ros.h>

#include <nodelet/loader.h>

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "controller");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "rci_controller/controller_nodelet", remap, nargv);

    ROS_INFO("Started controller.");

    ros::spin();

    return 0;
}

