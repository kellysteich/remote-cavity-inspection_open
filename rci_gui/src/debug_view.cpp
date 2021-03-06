//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/rci_gui/debug_view.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui
{

/*****************************************************************************
** Implementation
*****************************************************************************/
DebugView::DebugView(int argc, char** argv, std::string node_name) :
    BaseRosThread(argc, argv, node_name)
{
    frequency_ = 100;
    got_gt_cavity_ = false;
}

void DebugView::initRosComm(ros::NodeHandle n)
{
    ROS_INFO_STREAM("Init " << node_name_ << " thread: " << QThread::currentThreadId());

    robot_pose_sub_ = n.subscribe("gui/input/robot_pose", 8, &DebugView::robotPoseCb, this, ros::TransportHints().tcpNoDelay());
    desired_robot_pose_sub_ = n.subscribe("gui/input/desired_robot_pose", 8, &DebugView::desiredRobotPoseCb, this, ros::TransportHints().tcpNoDelay());
    gt_robot_pose_sub_ = n.subscribe("gui/input/gt_robot_pose", 8, &DebugView::gtRobotPoseCb, this, ros::TransportHints().tcpNoDelay());
    endeffector_pose_linkstates_sub_ = n.subscribe("gui/input/endeffector_pose/linkstates", 8, &DebugView::endeffectorPoseLinkstatesCb, this, ros::TransportHints().tcpNoDelay());
    endeffector_pose_odometry_sub_ = n.subscribe("gui/input/endeffector_pose/odometry", 8, &DebugView::endeffectorPoseOdometryCb, this, ros::TransportHints().tcpNoDelay());
    desired_endeffector_pose_sub_ = n.subscribe("gui/input/desired_endeffector_pose", 8, &DebugView::desiredEndeffectorPoseCb, this, ros::TransportHints().tcpNoDelay());
    gt_cavity_pose_linkstates_sub_ = n.subscribe("gui/input/gt_cavity_pose/linkstates", 8, &DebugView::gtCavityPoseLinkstatesCb, this, ros::TransportHints().tcpNoDelay());
    gt_cavity_pose_odometry_sub_ = n.subscribe("gui/input/gt_cavity_pose/odometry", 8, &DebugView::gtCavityPoseOdometryCb, this, ros::TransportHints().tcpNoDelay());
}

void DebugView::robotPoseCb(const nav_msgs::OdometryConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " robotPoseCb thread: " << QThread::currentThreadId());
    robot_x_.push_back(msg->pose.pose.position.x);
    robot_y_.push_back(msg->pose.pose.position.y);
    robot_z_.push_back(msg->pose.pose.position.z);
    robot_time_.push_back(msg->header.stamp.sec + 0.000000001*msg->header.stamp.nsec);

    // Inform GUI thread of new cavity poses (always in batches of 50, i.e at 4 Hz)
    if(robot_x_.size() == 50)
    {
        if (!restart_ && !finished_ && !abort_)
        {
            Q_EMIT newRobotPoses(robot_x_, robot_y_, robot_z_, robot_time_);
            if(!desired_robot_x_.isEmpty())
            {
                QVector<double> time;
                time.push_back(msg->header.stamp.sec + 0.000000001*msg->header.stamp.nsec);
                Q_EMIT newDesiredRobotPoses(desired_robot_x_, desired_robot_y_, desired_robot_z_, time);
            }
        }
        robot_x_.clear();
        robot_y_.clear();
        robot_z_.clear();
        robot_time_.clear();
    }
}

void DebugView::desiredRobotPoseCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " desiredRobotPoseCb thread: " << QThread::currentThreadId());
    desired_robot_x_.clear();
    desired_robot_y_.clear();
    desired_robot_z_.clear();

    desired_robot_x_.push_back(msg->pose.position.x);
    desired_robot_y_.push_back(msg->pose.position.y);
    desired_robot_z_.push_back(msg->pose.position.z);
}

void DebugView::gtRobotPoseCb(const nav_msgs::OdometryConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " gtRobotPoseCb thread: " << QThread::currentThreadId());

    robot_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    robot_or_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    mav_msgs::EigenOdometry robot_odom;
    mav_msgs::eigenOdometryFromMsg(*msg,&robot_odom);

		if(got_gt_cavity_)
		{
    Eigen::Vector3d cavity_pos_robot = gt_cavity_pos_- robot_pos_;
    cavity_pos_robot = robot_odom.orientation_W_B.toRotationMatrix().transpose() * cavity_pos_robot;

    gt_cavity_x_.push_back(cavity_pos_robot.x());
    gt_cavity_y_.push_back(cavity_pos_robot.y());
    gt_cavity_z_.push_back(cavity_pos_robot.z());
    gt_cavity_time_.push_back(msg->header.stamp.sec + 0.000000001*msg->header.stamp.nsec);
		}

    gt_robot_x_.push_back(msg->pose.pose.position.x);
    gt_robot_y_.push_back(msg->pose.pose.position.y);
    gt_robot_z_.push_back(msg->pose.pose.position.z);
    gt_robot_time_.push_back(msg->header.stamp.sec + 0.000000001*msg->header.stamp.nsec);

    // Inform GUI thread of new cavity poses (always in batches of 50, i.e at 4 Hz)
    if(gt_robot_x_.size() == 50)
    {
        if (!restart_ && !finished_ && !abort_)
        {
            Q_EMIT newGtRobotPoses(gt_robot_x_, gt_robot_y_, gt_robot_z_, gt_robot_time_);
						if(got_gt_cavity_)
            	Q_EMIT newGtCavityPoses(gt_cavity_x_, gt_cavity_y_, gt_cavity_z_, gt_cavity_time_);
        }
        gt_robot_x_.clear();
        gt_robot_y_.clear();
        gt_robot_z_.clear();
        gt_robot_time_.clear();
        gt_cavity_x_.clear();
        gt_cavity_y_.clear();
        gt_cavity_z_.clear();
        gt_cavity_time_.clear();
    }
}

void DebugView::endeffectorPoseLinkstatesCb(const gazebo_msgs::LinkStatesConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " endeffectorPoseCb thread: " << QThread::currentThreadId());

    Eigen::Vector3d endeffector_pos(msg->pose.back().position.x, msg->pose.back().position.y, msg->pose.back().position.z);
    endeffector_pos = endeffector_pos - robot_pos_;
    endeffector_pos = robot_or_.inverse() * endeffector_pos;

    endeffector_x_.push_back(endeffector_pos.x());
    endeffector_y_.push_back(endeffector_pos.y());
    endeffector_z_.push_back(endeffector_pos.z());
    endeffector_time_.push_back(ros::Time::now().sec + 0.000000001*ros::Time::now().nsec);

    // Inform GUI thread of new cavity poses (always in batches of 50, i.e at 4 Hz)
    if(endeffector_x_.size() == 500)
    {
        if (!restart_ && !finished_ && !abort_)
        {
            Q_EMIT newEndeffectorPoses(endeffector_x_, endeffector_y_, endeffector_z_, endeffector_time_);
            if(!desired_endeffector_x_.isEmpty())
            {
                QVector<double> time;
                time.push_back(ros::Time::now().sec + 0.000000001*ros::Time::now().nsec);
                Q_EMIT newDesiredEndeffectorPoses(desired_endeffector_x_, desired_endeffector_y_, desired_endeffector_z_, time);
            }
        }
        endeffector_x_.clear();
        endeffector_y_.clear();
        endeffector_z_.clear();
        endeffector_time_.clear();
    }
}

void DebugView::endeffectorPoseOdometryCb(const nav_msgs::OdometryConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " endeffectorPoseCb thread: " << QThread::currentThreadId());

    Eigen::Vector3d endeffector_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    endeffector_pos = endeffector_pos - robot_pos_;
    endeffector_pos = robot_or_.inverse() * endeffector_pos;

    endeffector_x_.push_back(endeffector_pos.x());
    endeffector_y_.push_back(endeffector_pos.y());
    endeffector_z_.push_back(endeffector_pos.z());
    endeffector_time_.push_back(msg->header.stamp.sec + 0.000000001*msg->header.stamp.nsec);

    // Inform GUI thread of new cavity poses (always in batches of 50, i.e at 4 Hz)
    if(endeffector_x_.size() == 50)
    {
        if (!restart_ && !finished_ && !abort_)
        {
            Q_EMIT newEndeffectorPoses(endeffector_x_, endeffector_y_, endeffector_z_, endeffector_time_);
            if(!desired_endeffector_x_.isEmpty())
            {
                QVector<double> time;
                time.push_back(msg->header.stamp.sec + 0.000000001*msg->header.stamp.nsec);
                Q_EMIT newDesiredEndeffectorPoses(desired_endeffector_x_, desired_endeffector_y_, desired_endeffector_z_, time);
            }
        }
        endeffector_x_.clear();
        endeffector_y_.clear();
        endeffector_z_.clear();
        endeffector_time_.clear();
    }
}

void DebugView::desiredEndeffectorPoseCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " desiredEndeffectorPoseCb thread: " << QThread::currentThreadId());
    desired_endeffector_x_.clear();
    desired_endeffector_y_.clear();
    desired_endeffector_z_.clear();

    desired_endeffector_x_.push_back(msg->pose.position.x);
    desired_endeffector_y_.push_back(msg->pose.position.y);
    desired_endeffector_z_.push_back(msg->pose.position.z);
}

void DebugView::gtCavityPoseOdometryCb(const nav_msgs::OdometryConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " gtCavityPoseCb thread: " << QThread::currentThreadId());

    gt_cavity_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    got_gt_cavity_ = true;
}

void DebugView::gtCavityPoseLinkstatesCb(const gazebo_msgs::LinkStatesConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " gtCavityPoseCb thread: " << QThread::currentThreadId());

    gt_cavity_pos_ = Eigen::Vector3d(msg->pose[1].position.x, msg->pose[1].position.y, msg->pose[1].position.z);
    got_gt_cavity_ = true;
}

}  // namespace rci_gui


