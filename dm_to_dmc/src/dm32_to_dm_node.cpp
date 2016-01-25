#include <ros/ros.h>

#include <nodelet/loader.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "dm32_to_dm_node");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "dm_to_dmc/dm32_to_dm_nodelet", remap, nargv);

    ROS_INFO("Started dm32_to_dm.");

    // Spin
    ros::spin();

    return 0;
}

