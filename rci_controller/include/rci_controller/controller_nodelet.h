//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef CONTROLLER_NODELET_H
#define CONTROLLER_NODELET_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <manipulator_msgs/CommandPositionServoMotor.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <rci_comm/StartController.h>
#include <rci_comm/LiftArm.h>
#include <rci_comm/GetControllerParams.h>
#include <rci_comm/SetControllerParams.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <omp.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_controller
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! controller nodelet,
 * gets a cavity position and robot pose and outputs a robot and arm trajectory
 */
class ControllerNodelet : public nodelet::Nodelet
{
public:

    /*! Constructor
     */
    ControllerNodelet();

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

    /*! The callback function for the ros subscriber pitching_jointstate_sub_
     */
    void jointstatePitchingCb(const sensor_msgs::JointStateConstPtr& msg);

    /*! The callback function for the ros subscriber right_jointstate_sub_
     */
    void jointstateRightCb(const sensor_msgs::JointStateConstPtr& msg);

    /*! The callback function for the ros subscriber left_jointstate_sub_
     */
    void jointstateLeftCb(const sensor_msgs::JointStateConstPtr& msg);

    /*! The callback function for the ros service start_controller_service_
     */
    bool startControllerCb(rci_comm::StartControllerRequest &req, rci_comm::StartControllerResponse &res);

    /*! The callback function for the ros service lift_arm_service_
     */
    bool liftArmCb(rci_comm::LiftArmRequest &req, rci_comm::LiftArmResponse &res);   

    /*! The callback function for the ros service get_params_service_,
     * where the parameters are sent back
     */
    bool getParams(rci_comm::GetControllerParamsRequest &req, rci_comm::GetControllerParamsResponse &res);

    /*! The callback function for the ros service set_params_service_,
     * where the parameters are changed
     */
    bool setParams(rci_comm::SetControllerParamsRequest &req, rci_comm::SetControllerParamsResponse &res);

    /*! The callback function for the ros timer update_timer_
     */
    void updateCb(const ros::TimerEvent& e);

    /*! Update the stats, i.e. avg_frequency
     */
    void updateStats();

    /*! Create a CommandPositionServoMotor Msg from a position
     */
    manipulator_msgs::CommandPositionServoMotor createCommandPositionServoMotorMsg(double position);

    /*! Create a PoseStamped Msg from an Eigen position Vector and orientation Quaternion
     */
    geometry_msgs::PoseStamped createPoseStampedMsgFromEigen(Eigen::Vector3d position, Eigen::Quaterniond orientation, std::string id);

    /*! Get the end effector position using forward kinematics with the joint angles q1 (left joint) and q2 (right joint)
     */
    Eigen::Vector3d forwardKinematics(double q1, double q2);

    ros::NodeHandle nh_, private_nh_;

    ros::Publisher arm_goal_pub_, robot_goal_pub_, joint_trajectory_pub_, pitching_pub_, left_pub_, right_pub_, robot_goal_debug_pub_;
    ros::Subscriber robot_pose_sub_, cavity_pose_sub_, pitching_jointstate_sub_, right_jointstate_sub_, left_jointstate_sub_;
    ros::ServiceServer start_controller_service_, lift_arm_service_, get_params_service_, set_params_service_;
    ros::Timer update_timer_;

    bool is_simulated_;

    Eigen::Quaterniond robot_orientation_, cavity_orientation_;
    Eigen::Vector3d robot_position_, cavity_position_, desired_cavity_position_;

    bool received_first_cavity_position_, do_control_;

    double pitching_angle_, right_angle_, left_angle_;

    int update_rate_;
    double desired_pitching_angle_, robot_dist_to_tree_;
    double robot_to_arm_translation_x_, robot_to_arm_translation_y_, robot_to_arm_translation_z_;
    Eigen::Vector3d robot_to_arm_translation_;
    double robot_offset_x_, robot_offset_y_, robot_offset_z_;
    Eigen::Vector3d robot_offset_;
    double cavity_depth_;

    double arm_pitching_tol_, robot_position_tol_x_, robot_position_tol_y_, robot_position_tol_z_, robot_orientation_tol_;

    Eigen::Vector3d prev_arm_command_;
    double filter_alpha_;

    double l1_, l3_, l5_, l6_, alpha_, x2_;

    boost::posix_time::ptime start_time_;
    std::vector<int> frequency_;
    int avg_frequency_;
};

}  // namespace rci_controller

#endif // CONTROLLER_NODELET_H
