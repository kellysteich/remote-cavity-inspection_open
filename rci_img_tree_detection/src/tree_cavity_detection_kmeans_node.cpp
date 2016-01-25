#include <ros/ros.h>

#include <nodelet/loader.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "tree_cavity_detection_kmeans_node");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "rci_img_tree_detection/tree_cavity_detection_kmeans_nodelet", remap, nargv);

    ROS_INFO("Started depth kmeans contour img tree detection.");

    // Spin
    ros::spin();

    return 0;
}

