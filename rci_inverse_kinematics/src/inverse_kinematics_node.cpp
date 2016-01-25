#include <ros/ros.h>

#include <nodelet/loader.h>

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "inverse_kinematics");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "rci_inverse_kinematics/inverse_kinematics_nodelet", remap, nargv);

    ROS_INFO("Started inverse_kinematics.");

    ros::spin();

    return 0;
}

