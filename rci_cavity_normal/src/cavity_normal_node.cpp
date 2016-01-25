#include <ros/ros.h>

#include <nodelet/loader.h>

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "cavity_normal");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "rci_cavity_normal/cavity_normal_nodelet", remap, nargv);

    ROS_INFO("Started cavity_normal.");

    ros::spin();

    return 0;
}

