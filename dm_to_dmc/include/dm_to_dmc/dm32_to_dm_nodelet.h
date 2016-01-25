//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef DM32_TO_DM_NODELET_H
#define DM32_TO_DM_NODELET_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dm_to_dmc
{

/*****************************************************************************
** Class
*****************************************************************************/

/*! Pc to Dm nodelet,
 * transforms a pointcloud to a depth map
 */
class Dm32ToDmNodelet : public nodelet::Nodelet
{
public:

    /*! Constructor
     */
    Dm32ToDmNodelet();

private:

    /*! All initialization of the ROS infrastructure
     */
    virtual void onInit();

    /*! The callback function for the ros subscriber sub_
     */
    void cb(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle nh_, private_nh_;

    image_transport::Publisher image_pub_;
    image_transport::Subscriber sub_;

    int sensor_depth_max_;

    float scale_;

    std::string sensor_name_;
};

}  // namespace dm_to_dmc

#endif // DM32_TO_DM_NODELET_H
