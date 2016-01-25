//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef FILTER_PIPELINE_NODELET_H
#define FILTER_PIPELINE_NODELET_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include <sensor_msgs/PointCloud2.h>

#include <rci_comm/GetFilterPipelineParams.h>
#include <rci_comm/SetFilterPipelineParams.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>

#include <omp.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_filter_pipeline
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! filter pipeline nodelet,
 * gets a point cloud and filters it
 */
class FilterPipelineNodelet : public nodelet::Nodelet
{
public:

    /*! Constructor
     */
    FilterPipelineNodelet();

private:

    /*! All initialization of the ROS infrastructure
     */
    virtual void onInit();


    /*! The callback function for the ros subscriber sub_
     */
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


    /*! The callback function for the ros service get_params_service_,
     * where the parameters are sent back
     */
    bool getParams(rci_comm::GetFilterPipelineParamsRequest &req, rci_comm::GetFilterPipelineParamsResponse &res);


    /*! The callback function for the ros service set_params_service_,
     * where the parameters are changed
     */
    bool setParams(rci_comm::SetFilterPipelineParamsRequest &req, rci_comm::SetFilterPipelineParamsResponse &res);


    /*! Update the stats, i.e. avg_frequency
     */
    void updateStats();


    ros::NodeHandle nh_, private_nh_;

    ros::Subscriber sub_;
    ros::Publisher pub_;

    ros::ServiceServer get_params_service_, set_params_service_;

    bool use_passthrough_, use_voxelgrid_, use_radial_;

    double passthrough_min_, passthrough_max_;

    double leafsize_;

    int min_neighbors_;
    double radius_search_;

    boost::posix_time::ptime start_time_;
    std::vector<int> frequency_;
    int avg_frequency_;

    boost::chrono::nanoseconds time_all_, time_pass_, time_vox_, time_rad_;
    int count_time_;
};

}  // namespace rci_filter_pipeline

#endif // FILTER_PIPELINE_NODELET_H

