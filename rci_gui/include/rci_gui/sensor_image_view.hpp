//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef SENSOR_IMAGE_VIEW_HPP
#define SENSOR_IMAGE_VIEW_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include "base_ros_thread.hpp"

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>

#include <QImage>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! Sensor Image View node,
 * gets the sensor images (depth images) and outputs them to the GUI
 */
class SensorImageView : public BaseRosThread
{
    Q_OBJECT

public:

    /*! Constructor
     */
    SensorImageView(int argc, char** argv, std::string node_name);

    /*! The callback function for the ros subscriber image_sub_, where the sensor image is grabed and outputed
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    /*! The callback function for the ros subscriber lost_sub_
     */
    void lostCb(const std_msgs::Bool& msg);

    /*! Converts a ros sensor_msgs::Image to a cv::Mat
     */
    cv::Mat rosImgToCvMat(sensor_msgs::ImageConstPtr ros_img, const std::string encoding, bool share);

    /*! Converts a cv::Mat to a QImage
     */
    QImage cvMatToQImage( const cv::Mat &in_mat );


Q_SIGNALS:

    /*! Notifies the GUI that there is a new output image
     */
    void newFrame(const QImage &frame, const int &frame_width, const int &frame_height);

    /*! Notifies the GUI that the tree has been lost
     */
    void lostTree();

    /*! Notifies the GUI that the tree has been found
     */
    void foundTree();

private:

    /*! Initializes the ros subscriber image_sub_
     */
    void initRosComm(ros::NodeHandle n);

    image_transport::Subscriber image_sub_;
    ros::Subscriber lost_sub_;
};

}  // namespace rci_gui

#endif // SENSOR_IMAGE_VIEW_HPP
