#include <ros/ros.h>

#include <nodelet/loader.h>

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "filter_pipeline_node");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "rci_filter_pipeline/filter_pipeline_nodelet", remap, nargv);

    ROS_INFO("Started filter pipeline.");

    // Spin
    ros::spin();

    return 0;
}

