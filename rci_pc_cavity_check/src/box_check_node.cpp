#include <ros/ros.h>

#include <nodelet/loader.h>

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "box_check");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "rci_pc_cavity_check/box_check_nodelet", remap, nargv);

    ROS_INFO("Started box check.");

    ros::spin();

    return 0;
}

