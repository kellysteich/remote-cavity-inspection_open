//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/rci_gui/sensor_image_view.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui
{

/*****************************************************************************
** Implementation
*****************************************************************************/

SensorImageView::SensorImageView(int argc, char** argv, std::string node_name) :
    BaseRosThread(argc, argv, node_name)
{
    frequency_ = 10;
    rateGui_ = 1;
}

void SensorImageView::initRosComm(ros::NodeHandle n)
{
    ROS_INFO_STREAM("Init " << node_name_ << " thread: " << QThread::currentThreadId());

    image_transport::ImageTransport it(n);
    image_sub_ = it.subscribe("gui/input/image/depth", 1, &SensorImageView::imageCb, this);
    lost_sub_ = n.subscribe("gui/lost_tree", 1, &SensorImageView::lostCb, this);
}

void SensorImageView::lostCb(const std_msgs::Bool &msg)
{
    if(msg.data)
        Q_EMIT lostTree();
    else
        Q_EMIT foundTree();
}

void SensorImageView::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM_ONCE(node_name_ << " cb thread: " << QThread::currentThreadId());

    // Inform GUI thread of new frame (QImage) at a rate of rateGui
    if(nFrames_ % rateGui_ == 0)
    {
        //convert ROS image message to a CvImage.
        cv::Mat cvframe = rosImgToCvMat(msg, sensor_msgs::image_encodings::BGR8, false);

        //Convert opencv image to QImage
        QImage qframe = cvMatToQImage(cvframe);

        if (!restart_ && !finished_ && !abort_)
            Q_EMIT newFrame(qframe, cvframe.cols, cvframe.rows);
    }
    nFrames_++;
}

cv::Mat SensorImageView::rosImgToCvMat(sensor_msgs::ImageConstPtr ros_img, const std::string encoding, bool share)
{
    //convert ROS image message to a CvImage.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if(share)
            cv_ptr = cv_bridge::toCvShare(ros_img, encoding);
        else
            cv_ptr = cv_bridge::toCvCopy(ros_img, encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge input exception: %s", e.what());
        std::exit;
    }
    return cv_ptr->image;
}

QImage SensorImageView::cvMatToQImage( const cv::Mat &in_mat )
{
    switch ( in_mat.type() )
    {
    // 8-bit, 4 channel
    case CV_8UC4:
    {
        QImage image( in_mat.data, in_mat.cols, in_mat.rows, in_mat.step, QImage::Format_RGB32 );
        return image;
    }

        // 8-bit, 3 channel
    case CV_8UC3:
    {
        QImage image( in_mat.data, in_mat.cols, in_mat.rows, in_mat.step, QImage::Format_RGB888 );
        return image.rgbSwapped();
    }

        // 8-bit, 1 channel
    case CV_8UC1:
    {
        static QVector<QRgb>  s_color_table;
        // only create our color table once
        if ( s_color_table.isEmpty() )
        {
            for ( int i = 0; i < 256; ++i )
                s_color_table.push_back( qRgb( i, i, i ) );
        }
        QImage image( in_mat.data, in_mat.cols, in_mat.rows, in_mat.step, QImage::Format_Indexed8 );
        image.setColorTable( s_color_table );
        return image;
    }

    default:
        log(Error,std::string("ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" + in_mat.type()));
        break;
    }

    return QImage();
}

}  // namespace rci_gui
