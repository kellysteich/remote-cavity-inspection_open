#include <dm32_to_dm_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace dm_to_dmc
{

Dm32ToDmNodelet::Dm32ToDmNodelet()
{
}

void Dm32ToDmNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    nh_.param("sensor/name", sensor_name_, std::string("realsenseR200"));

    image_transport::ImageTransport it(nh_);
    image_pub_ = it.advertise("dm32_to_dm/output", 1);
    sub_ = it.subscribe("dm32_to_dm/input", 1, &Dm32ToDmNodelet::cb, this);

    scale_ = 0.001;

    if(sensor_name_ == "realsenseF200")
    {
        sensor_depth_max_ = 1200;
    }
    else if(sensor_name_ == "realsenseR200")
    {
        private_nh_.param("sensor_depth_max", sensor_depth_max_, int(4000));
    }
    else if(sensor_name_ == "picoflexx")
    {
        nh_.param("sensor/depth/max", sensor_depth_max_, int(5000));
    }
    else {
        NODELET_ERROR("Wrong sensor name!");
        return;
    }

    NODELET_INFO_STREAM("Initialized dm32 to dm nodelet for sensor " << sensor_name_
                        << ", with sensor_depth_max=" << sensor_depth_max_);
}

void Dm32ToDmNodelet::cb(const sensor_msgs::ImageConstPtr &msg)
{
    //convert ROS image message to a CvImage.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        NODELET_ERROR("cv_bridge input exception: %s", e.what());
        return;
    }

    cv::Mat cvframe = cv::Mat::zeros(cv_ptr->image.size(), CV_16UC1);

    for (int i = 0; i < cv_ptr->image.rows; i++)
    { //rows
        for (int j = 0; j < cv_ptr->image.cols; j++)
        { //cols
            float depth = cv_ptr->image.at<float>(i,j);
            depth = depth / scale_;
            cvframe.at<unsigned short>(i,j) = (unsigned short) depth;
        }
    }

    //convert opencv image to ros image and publish it
    cv_bridge::CvImagePtr out_msg_ptr(new cv_bridge::CvImage);
    out_msg_ptr->header = msg->header;
    out_msg_ptr->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    out_msg_ptr->image = cvframe;
    image_pub_.publish(out_msg_ptr->toImageMsg());
}

} //namespace dm_to_dmc

PLUGINLIB_DECLARE_CLASS(dm_to_dmc, Dm32ToDmNodelet, dm_to_dmc::Dm32ToDmNodelet, nodelet::Nodelet);


