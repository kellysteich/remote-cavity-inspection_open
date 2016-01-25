//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef CAVITY_NORMAL_NODELET_H
#define CAVITY_NORMAL_NODELET_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <rci_comm/CavityBox.h>
#include <rci_comm/CavityBoxNormal.h>

#include <rci_comm/GetCavityNormalParams.h>
#include <rci_comm/SetCavityNormalParams.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>
#include <Eigen/Geometry>

#include <omp.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_cavity_normal
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! Cavity Normal nodelet,
 * gets a cavity position and calculates its normal by looking at its surrounding points
 */
class CavityNormalNodelet : public nodelet::Nodelet
{
public:

    /*! Constructor
     */
    CavityNormalNodelet();

private:

    /*! All initialization of the ROS infrastructure
     */
    virtual void onInit();

    /*! The callback function for the ros time synchronizer sync_,
     * where the new normal is conmputed
     */
    void cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const rci_comm::CavityBoxConstPtr& cavity_msg);

    /*! Update the stats, i.e. avg_frequency
     */
    void updateStats();

    /*! The callback function for the ros service get_params_service_,
     * where the parameters are sent back
     */
    bool getParams(rci_comm::GetCavityNormalParamsRequest &req, rci_comm::GetCavityNormalParamsResponse &res);


    /*! The callback function for the ros service set_params_service_,
     * where the parameters are changed
     */
    bool setParams(rci_comm::SetCavityNormalParamsRequest &req, rci_comm::SetCavityNormalParamsResponse &res);


    ros::NodeHandle nh_, private_nh_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<rci_comm::CavityBox> cavity_sub_;
    boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, rci_comm::CavityBox> > sync_;

    ros::Publisher pub_, cavity_pose_pub_;

    ros::ServiceServer get_params_service_, set_params_service_;

    boost::posix_time::ptime start_time_;
    std::vector<int> frequency_;
    int avg_frequency_;

    boost::chrono::nanoseconds time_all_;
    int count_time_;

    double band_width_;
    double octree_resolution_;
};

}  // namespace rci_cavity_normal

#endif // CAVITY_NORMAL_NODELET_H
