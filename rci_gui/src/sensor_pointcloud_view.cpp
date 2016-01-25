//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/rci_gui/sensor_pointcloud_view.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

SensorPointcloudView::SensorPointcloudView(int argc, char** argv, std::string node_name) :
    BaseRosThread(argc, argv, node_name)
{
    frequency_ = 30;
    rateGui_ = 1;
}

void SensorPointcloudView::initRosComm(ros::NodeHandle n)
{
    ROS_INFO_STREAM("Init " << node_name_ << " thread: " << QThread::currentThreadId());

    cloud_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2> > ();
    cloud_sub_->subscribe(n, "gui/input/points", 4);
    tree_sub_ = boost::make_shared<message_filters::Subscriber<rci_comm::Tree> > ();
    tree_sub_->subscribe(n, "gui/input/tree", 4);
    cavity_sub_ = boost::make_shared<message_filters::Subscriber<rci_comm::CavityBoxNormal> > ();
    cavity_sub_->subscribe(n, "gui/input/cavity_box", 4);
    sync_ = boost::make_shared<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, rci_comm::Tree, rci_comm::CavityBoxNormal> > (8);
    sync_->connectInput(*cloud_sub_.get(), *tree_sub_.get(), *cavity_sub_.get());
    sync_->registerCallback (bind (&SensorPointcloudView::cb, this, _1, _2, _3));
}

void SensorPointcloudView::cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                              const rci_comm::TreeConstPtr &tree_msg,
                              const rci_comm::CavityBoxNormalConstPtr &cavity_msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " cb thread: " << QThread::currentThreadId());

    // Inform GUI thread of new frame (Pointcloud) at a rate of rateGui
    if(nFrames_ % rateGui_ == 0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        QVector3D min_pt_tree = QVector3D(tree_msg->tree_min.x, tree_msg->tree_min.y, tree_msg->tree_min.z);
        QVector3D max_pt_tree = QVector3D(tree_msg->tree_max.x, tree_msg->tree_max.y, tree_msg->tree_max.z);

        QVector3D center(0,0,0);
        if(cavity_msg->center.x != 0 && cavity_msg->center.y != 0 && cavity_msg->center.z != 0)
            center = QVector3D(cavity_msg->center.x, cavity_msg->center.y, cavity_msg->center.z);

        QVector3D min_pt_cavity = QVector3D(cavity_msg->min_x, cavity_msg->min_y, cavity_msg->min_z);
        QVector3D max_pt_cavity = QVector3D(cavity_msg->max_x, cavity_msg->max_y, cavity_msg->max_z);

        QVector3D cavity_normal = QVector3D(cavity_msg->normal.x, cavity_msg->normal.y, cavity_msg->normal.z);
        float radius = cavity_msg->radius;

        if (!restart_ && !finished_ && !abort_)
            Q_EMIT newPointcloud(cloud, min_pt_tree, max_pt_tree, center, min_pt_cavity, max_pt_cavity, cavity_normal, radius);
    }
    nFrames_++;
}

}  // namespace rci_gui

