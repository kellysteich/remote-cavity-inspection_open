//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef Box_CHECK_NODELET_H
#define Box_CHECK_NODELET_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <rci_comm/CavityBox.h>

#include <rci_comm/GetBoxCheckParams.h>
#include <rci_comm/SetBoxCheckParams.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/octree/octree.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>

#include <omp.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_pc_cavity_check
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! Box Check nodelet,
 * gets an approx cavity position and finds the largest box that fits into this cavity
 */
class BoxCheckNodelet : public nodelet::Nodelet
{
public:

    /*! Constructor
     */
    BoxCheckNodelet();

private:

    /*! All initialization of the ROS infrastructure
     */
    virtual void onInit();

    /*! The callback function for the ros time synchronizer sync_,
     * where the new cavity point is checked in the pointcloud
     */
    void cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const geometry_msgs::PointStampedConstPtr& cavity_msg);

    /*! Update the stats, i.e. avg_frequency
     */
    void updateStats();

    /*! The callback function for the ros service get_params_service_,
     * where the parameters are sent back
     */
    bool getParams(rci_comm::GetBoxCheckParamsRequest &req, rci_comm::GetBoxCheckParamsResponse &res);


    /*! The callback function for the ros service set_params_service_,
     * where the parameters are changed
     */
    bool setParams(rci_comm::SetBoxCheckParamsRequest &req, rci_comm::SetBoxCheckParamsResponse &res);


    ros::NodeHandle nh_, private_nh_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<geometry_msgs::PointStamped> cavity_sub_;
    boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PointStamped> > sync_;

    ros::Publisher pub_;

    ros::ServiceServer get_params_service_, set_params_service_;

    boost::posix_time::ptime start_time_;
    std::vector<int> frequency_;
    int avg_frequency_;

    double octree_resolution_, initial_step_;
    double start_box_width_, start_box_height_, start_box_length_;
    int max_n_inliers_;

    boost::chrono::nanoseconds time_all_, time_start_, time_x_, time_y_, time_z_;
    int count_time_, count_start_, count_x_, count_y_, count_z_;
};

}  // namespace rci_pc_cavity_check

#endif // Box_CHECK_NODELET_H
