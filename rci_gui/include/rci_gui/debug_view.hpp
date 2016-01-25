#ifndef DEBUG_VIEW_HPP
#define DEBUG_VIEW_HPP
//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
/*****************************************************************************
** Includes
*****************************************************************************/

#include "base_ros_thread.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

#include <Eigen/Geometry>

#include <QVector>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! Cavity Pose node,
 * gets the (estimated) cavity pose (in robot frame) and outputs it to the GUI
 */
class DebugView : public BaseRosThread
{
    Q_OBJECT

public:

    /*! Constructor
     */
    DebugView(int argc, char** argv, std::string node_name);

    /*! The callback function for the ros subscriber robot_pose_sub_
     */
    void robotPoseCb(const nav_msgs::OdometryConstPtr& msg);

    /*! The callback function for the ros subscriber desired_robot_pose_sub_
     */
    void desiredRobotPoseCb(const geometry_msgs::PoseStampedConstPtr& msg);

    /*! The callback function for the ros subscriber gt_robot_pose_sub_
     */
    void gtRobotPoseCb(const nav_msgs::OdometryConstPtr& msg);

    /*! The callback function for the ros subscriber endeffector_pose_linkstates_sub_
     */
    void endeffectorPoseLinkstatesCb(const gazebo_msgs::LinkStatesConstPtr& msg);

    /*! The callback function for the ros subscriber endeffector_pose_odometry_sub_
     */
    void endeffectorPoseOdometryCb(const nav_msgs::OdometryConstPtr& msg);

    /*! The callback function for the ros subscriber desired_endeffector_pose_sub_
     */
    void desiredEndeffectorPoseCb(const geometry_msgs::PoseStampedConstPtr& msg);

    /*! The callback function for the ros subscriber gt_cavity_pose_linkstates_sub_
     */
    void gtCavityPoseLinkstatesCb(const gazebo_msgs::LinkStatesConstPtr& msg);

    /*! The callback function for the ros subscriber gt_cavity_pose_linkstates_sub_
     */
    void gtCavityPoseOdometryCb(const nav_msgs::OdometryConstPtr& msg);


Q_SIGNALS:

    /*! Notifies the GUI that there is a new batch of output robot poses
     */
    void newRobotPoses(const QVector<double> &robot_x, const QVector<double> &robot_y, const QVector<double> &robot_z, const QVector<double> &robot_time);

    /*! Notifies the GUI that there is a new batch of output desired robot poses
     */
    void newDesiredRobotPoses(const QVector<double> &robot_x, const QVector<double> &robot_y, const QVector<double> &robot_z, const QVector<double> &robot_time);

    /*! Notifies the GUI that there is a new batch of output ground truth robot poses
     */
    void newGtRobotPoses(const QVector<double> &robot_x, const QVector<double> &robot_y, const QVector<double> &robot_z, const QVector<double> &robot_time);

    /*! Notifies the GUI that there is a new batch of output endeffector poses
     */
    void newEndeffectorPoses(const QVector<double> &robot_x, const QVector<double> &robot_y, const QVector<double> &robot_z, const QVector<double> &robot_time);

    /*! Notifies the GUI that there is a new batch of output desired endeffector poses
     */
    void newDesiredEndeffectorPoses(const QVector<double> &robot_x, const QVector<double> &robot_y, const QVector<double> &robot_z, const QVector<double> &robot_time);

    /*! Notifies the GUI that there is a new batch of output ground truth cavity poses
     */
    void newGtCavityPoses(const QVector<double> &robot_x, const QVector<double> &robot_y, const QVector<double> &robot_z, const QVector<double> &robot_time);

private:

    /*! Initializes the ros subscriber image_sub_
     */
    void initRosComm(ros::NodeHandle n);


    ros::Subscriber robot_pose_sub_, desired_robot_pose_sub_, gt_robot_pose_sub_;
    ros::Subscriber endeffector_pose_linkstates_sub_, endeffector_pose_odometry_sub_, desired_endeffector_pose_sub_, gt_cavity_pose_linkstates_sub_, gt_cavity_pose_odometry_sub_;

    QVector<double> robot_x_, robot_y_, robot_z_, robot_time_;
    QVector<double> desired_robot_x_, desired_robot_y_, desired_robot_z_;
    QVector<double> gt_robot_x_, gt_robot_y_, gt_robot_z_, gt_robot_time_;
    QVector<double> endeffector_x_, endeffector_y_, endeffector_z_, endeffector_time_;
    QVector<double> desired_endeffector_x_, desired_endeffector_y_, desired_endeffector_z_;
    QVector<double> gt_cavity_x_, gt_cavity_y_, gt_cavity_z_, gt_cavity_time_;

    Eigen::Vector3d robot_pos_;
    Eigen::Quaterniond robot_or_;
    Eigen::Vector3d gt_cavity_pos_;
    bool got_gt_cavity_;
};

}  // namespace rci_gui

#endif // DEBUG_VIEW_HPP
