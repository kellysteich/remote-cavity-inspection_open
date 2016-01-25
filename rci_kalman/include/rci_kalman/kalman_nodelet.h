//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef KALMAN_NODELET_H
#define KALMAN_NODELET_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <rci_comm/ResetKalman.h>
#include <rci_comm/GetKalmanParams.h>
#include <rci_comm/SetKalmanParams.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>
#include <Eigen/Geometry>

#include <omp.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_kalman
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! kalman nodelet,
 * gets a cavity position and robot pose and uses itin a kalman filter
 */
class KalmanNodelet : public nodelet::Nodelet
{
public:

    /*! Constructor
     */
    KalmanNodelet();

private:

    /*! All initialization of the ROS infrastructure
     */
    virtual void onInit();

    /*! The callback function for the ros subscriber robot_pose_sub_
     */
    void robotCb(const nav_msgs::OdometryConstPtr& msg);

    /*! The callback function for the ros subscriber cavity_pose_sub_
     */
    void cavityCb(const geometry_msgs::PoseStampedConstPtr& msg);

    /*! The callback function for the ros service reset_kalman_service_
     */
    bool resetKalmanCb(rci_comm::ResetKalmanRequest &req, rci_comm::ResetKalmanResponse &res);

    /*! The callback function for the ros service get_params_service_,
     * where the parameters are sent back
     */
    bool getParams(rci_comm::GetKalmanParamsRequest &req, rci_comm::GetKalmanParamsResponse &res);

    /*! The callback function for the ros service set_params_service_,
     * where the parameters are changed
     */
    bool setParams(rci_comm::SetKalmanParamsRequest &req, rci_comm::SetKalmanParamsResponse &res);

    /*! Update the stats, i.e. avg_frequency
     */
    void updateStats();

    ros::NodeHandle nh_, private_nh_;

    ros::Publisher pub_;
    ros::Subscriber robot_pose_sub_, cavity_pose_sub_;
    ros::ServiceServer reset_kalman_service_, get_params_service_, set_params_service_;

    boost::posix_time::ptime start_time_;
    std::vector<int> frequency_;
    int avg_frequency_;

    boost::chrono::nanoseconds time_all_;
    int count_time_;

    double qx_, qy_, qz_, rx_, ry_, rz_;
    Eigen::Matrix3d kalman_q_, kalman_r_;
    bool fixed_kalman_q_;

    double camera_to_robot_translation_x_, camera_to_robot_translation_y_, camera_to_robot_translation_z_;
    double camera_to_robot_rotation_r_, camera_to_robot_rotation_p_, camera_to_robot_rotation_y_;
    Eigen::Vector3d camera_to_robot_translation_;
    Eigen::Quaterniond camera_to_robot_rotation_;

    Eigen::Vector3d measured_cavity_position_robot_;
    Eigen::Quaterniond measured_cavity_orientation_robot_;

    bool got_first_robot_pose_;
    Eigen::Matrix3d robot_rotation_kminus1_;
    Eigen::Vector3d robot_translation_kminus1_;

    bool got_first_cavity_measurement_;
    Eigen::Vector3d estimated_cavity_position_robot_;
    Eigen::Matrix3d cavity_position_covariance_;
};

}  // namespace rci_kalman

#endif // KALMAN_NODELET_H
