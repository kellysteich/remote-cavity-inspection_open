#ifndef CAVITY_POSE_VIEW_H
#define CAVITY_POSE_VIEW_H
//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
/*****************************************************************************
** Includes
*****************************************************************************/

#include "base_ros_thread.hpp"

#include <geometry_msgs/PoseStamped.h>
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
class CavityPoseView : public BaseRosThread
{
    Q_OBJECT

public:

    /*! Constructor
     */
    CavityPoseView(int argc, char** argv, std::string node_name);

    /*! The callback function for the ros subscriber sub_
     */
    void cb(const geometry_msgs::PoseStampedConstPtr& msg);


Q_SIGNALS:

    /*! Notifies the GUI that there is a new batch of output cavity poses
     */
    void newPoses(const QVector<double> &cavity_x, const QVector<double> &cavity_y, const QVector<double> &cavity_z, const QVector<double> &time);

    /*! Notifies the GUI that there is a new output cavity pose
     */
    void newPose(const double &cavity_x, const double &cavity_y, const double &cavity_z);

private:

    /*! Initializes the ros subscriber image_sub_
     */
    void initRosComm(ros::NodeHandle n);


    ros::Subscriber sub_;

    QVector<double> cavity_x_, cavity_y_, cavity_z_, time_;

    double camera_to_robot_translation_x_, camera_to_robot_translation_y_, camera_to_robot_translation_z_;
    double camera_to_robot_rotation_r_, camera_to_robot_rotation_p_, camera_to_robot_rotation_y_;
    Eigen::Vector3d camera_to_robot_translation_;
    Eigen::Quaterniond camera_to_robot_rotation_;
};

}  // namespace rci_gui

#endif // CAVITY_POSE_VIEW_H
