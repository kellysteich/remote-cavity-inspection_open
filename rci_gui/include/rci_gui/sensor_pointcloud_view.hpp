//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef SENSOR_POINTCLOUD_VIEW_HPP
#define SENSOR_POINTCLOUD_VIEW_HPP
/*****************************************************************************
** Includes
*****************************************************************************/

#include "base_ros_thread.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <rci_comm/Tree.h>
#include <rci_comm/CavityBoxNormal.h>

#include <QVector3D>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! Pointcloud view node,
 *  gets the pointcloud, tree box, cavity box, cavity normal and outputs it to the GUI
 */
class SensorPointcloudView : public BaseRosThread
{
    Q_OBJECT

public:

    /*! Constructor
     */
    SensorPointcloudView(int argc, char** argv, std::string node_name);

    /*! The callback function for the ros time synchronizer sync_,
     *  where the pointcloud is grabed together with the tree and cavity box and cavity normal and outputed
     */
    void cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
            const rci_comm::TreeConstPtr& tree_msg,
            const rci_comm::CavityBoxNormalConstPtr& box_msg);

Q_SIGNALS:

    /*! Notifies the GUI that there is a new output pointcloud with tree and cavity boxes and normal
     */
    void newPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc,
                       const QVector3D &min_pt_tree, const QVector3D &max_pt_tree, const QVector3D &cavity_center,
                       const QVector3D &min_pt_cavity, const QVector3D &max_pt_cavity, const QVector3D &cavity_normalm, const float &radius);

private:

    /*! Initializes the ros subscriber sub_
     */
    void initRosComm(ros::NodeHandle n);

    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > cloud_sub_;
    boost::shared_ptr<message_filters::Subscriber<rci_comm::Tree> > tree_sub_;
    boost::shared_ptr<message_filters::Subscriber<rci_comm::CavityBoxNormal> > cavity_sub_;
    boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, rci_comm::Tree, rci_comm::CavityBoxNormal> > sync_;
};

}  // namespace rci_gui

#endif // SENSOR_POINTCLOUD_VIEW_HPP
