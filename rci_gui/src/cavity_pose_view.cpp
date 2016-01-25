//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/rci_gui/cavity_pose_view.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui
{

/*****************************************************************************
** Implementation
*****************************************************************************/

CavityPoseView::CavityPoseView(int argc, char** argv, std::string node_name) :
    BaseRosThread(argc, argv, node_name)
{
    frequency_ = 100;
}

void CavityPoseView::initRosComm(ros::NodeHandle n)
{
    ROS_INFO_STREAM("Init " << node_name_ << " thread: " << QThread::currentThreadId());

    n.param("sensor/translation/x", camera_to_robot_translation_x_, double(0.145));
    n.param("sensor/translation/y", camera_to_robot_translation_y_, double(0.0));
    n.param("sensor/translation/z", camera_to_robot_translation_z_, double(-0.07));
    n.param("sensor/rotation/r", camera_to_robot_rotation_r_, double(0.0));
    n.param("sensor/rotation/p", camera_to_robot_rotation_p_, double(0.0));
    n.param("sensor/rotation/y", camera_to_robot_rotation_y_, double(0.0));

    camera_to_robot_translation_ << camera_to_robot_translation_x_ , camera_to_robot_translation_y_ , camera_to_robot_translation_z_;

    Eigen::AngleAxisd rollAngle(camera_to_robot_rotation_r_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(camera_to_robot_rotation_p_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(camera_to_robot_rotation_y_, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d camera_to_robot_coordinate_frame_rotation;
    camera_to_robot_coordinate_frame_rotation << 0 , 0 , 1,
                                                -1 , 0 , 0,
                                                 0 , -1, 0;

    camera_to_robot_rotation_ = q * Eigen::Quaterniond(camera_to_robot_coordinate_frame_rotation);

    sub_ = n.subscribe("gui/input/cavity_pose", 1, &CavityPoseView::cb, this, ros::TransportHints().tcpNoDelay());
}

void CavityPoseView::cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " cb thread: " << QThread::currentThreadId());

    Eigen::Vector3d new_pose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    //compute estimated cavity pose in camera frame
    new_pose = new_pose - camera_to_robot_translation_;
    new_pose = camera_to_robot_rotation_.inverse() * new_pose;

    if (!restart_ && !finished_ && !abort_)
        Q_EMIT newPose(new_pose.x(), new_pose.y(), new_pose.z());

    cavity_x_.push_back(msg->pose.position.x);
    cavity_y_.push_back(msg->pose.position.y);
    cavity_z_.push_back(msg->pose.position.z);
    time_.push_back(msg->header.stamp.sec + 0.000000001*msg->header.stamp.nsec);

    // Inform GUI thread of new cavity poses (always in batches of 25, i.e at 4 Hz)
    if(cavity_x_.size() == 25)
    {
        if (!restart_ && !finished_ && !abort_)
            Q_EMIT newPoses(cavity_x_, cavity_y_, cavity_z_, time_);
        cavity_x_.clear();
        cavity_y_.clear();
        cavity_z_.clear();
        time_.clear();
    }
}

}  // namespace rci_gui

