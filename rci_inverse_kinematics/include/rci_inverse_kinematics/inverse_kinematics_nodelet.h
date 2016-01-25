//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef INVERSE_KINEMATICS_NODELET_H
#define INVERSE_KINEMATICS_NODELET_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include <geometry_msgs/PoseStamped.h>
#include <manipulator_msgs/CommandPositionServoMotor.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <Eigen/Geometry>

#include <omp.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_inverse_kinematics
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! inverse kinematics nodelet,
 * gets a cavity position and using inverse kinematics calculates the angle inputs of the robot arm, needed to reach that pose
 */
class InverseKinematicsNodelet : public nodelet::Nodelet
{
public:

    /*! Constructor
     */
    InverseKinematicsNodelet();

private:

    /*! All initialization of the ROS infrastructure
     */
    virtual void onInit();

    /*! The callback function for the ros subscriber endeffector_pose_sub_
     */
    void endeffectorCb(const geometry_msgs::PoseStampedConstPtr& msg);

    /*! Update the stats, i.e. avg_frequency
     */
    void updateStats();

    double dist2Points(double x1, double y1, double x2, double y2);
    double cross(double x1, double y1, double x2, double y2);
    double dot(double x1, double y1, double x2, double y2);
    double angle3Points(double ba_x, double ba_y, double bc_x, double bc_y);

    ros::NodeHandle nh_, private_nh_;

    ros::Publisher left_pub_, right_pub_, joint_trajectory_pub_;
    ros::Subscriber endeffector_pose_sub_;

    bool is_simulated_;

    boost::posix_time::ptime start_time_;
    std::vector<int> frequency_;
    int avg_frequency_;

    double l1_, l3_, l5_, l6_, alpha_;
    double x1_, y1_, x2_, y2_;
    double robot_to_arm_translation_x_, robot_to_arm_translation_y_, robot_to_arm_translation_z_; //up
    int n_semicircle_;

    Eigen::Vector3d robot_to_arm_translation_;
    Eigen::Quaterniond robot_to_arm_rotation_;

    Eigen::VectorXd semicircle_angles_;
};

}  // namespace rci_inverse_kinematics

#endif // INVERSE_KINEMATICS_NODELET_H
