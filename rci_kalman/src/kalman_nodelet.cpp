#include <kalman_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rci_kalman
{

KalmanNodelet::KalmanNodelet()
    : got_first_robot_pose_(false),
      got_first_cavity_measurement_(false),
      avg_frequency_(0),
      start_time_(boost::posix_time::microsec_clock::local_time()),
      measured_cavity_position_robot_(Eigen::Vector3d::Zero()),
      cavity_position_covariance_(Eigen::Matrix3d::Identity())
{
}

void KalmanNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    nh_.param("kalman/q/x", qx_, double(0.0));
    nh_.param("kalman/q/y", qy_, double(0.0));
    nh_.param("kalman/q/z", qz_, double(0.0));
    nh_.param("kalman/r/x", rx_, double(10));
    nh_.param("kalman/r/y", ry_, double(10));
    nh_.param("kalman/r/z", rz_, double(10));

    kalman_r_ << rx_ ,   0 ,   0 ,
                   0 , ry_ ,   0 ,
                   0 ,   0 , rz_ ;

    if(qx_ == 0)
    {
        fixed_kalman_q_ = false;
        kalman_q_ = Eigen::Matrix3d::Zero();
    }
    else
    {
        fixed_kalman_q_ = true;
        kalman_q_ << qx_ ,   0 ,   0 ,
                      0 , qy_ ,   0 ,
                      0 ,   0 , qz_ ;
    }

    nh_.param("sensor/translation/x", camera_to_robot_translation_x_, double(0.145));
    nh_.param("sensor/translation/y", camera_to_robot_translation_y_, double(0.0));
    nh_.param("sensor/translation/z", camera_to_robot_translation_z_, double(-0.07));
    nh_.param("sensor/rotation/r", camera_to_robot_rotation_r_, double(0.0));
    nh_.param("sensor/rotation/p", camera_to_robot_rotation_p_, double(0.1));
    nh_.param("sensor/rotation/y", camera_to_robot_rotation_y_, double(0.0));

    camera_to_robot_translation_ << camera_to_robot_translation_x_ , camera_to_robot_translation_y_ , camera_to_robot_translation_z_;

    Eigen::AngleAxisd rollAngle(camera_to_robot_rotation_r_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(camera_to_robot_rotation_p_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(camera_to_robot_rotation_y_, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

    Eigen::Matrix3d camera_to_robot_coordinate_frame_rotation;
    camera_to_robot_coordinate_frame_rotation << 0 , 0 , 1,
                                                -1 , 0 , 0,
                                                 0 , -1, 0;

    camera_to_robot_rotation_ = q * Eigen::Quaterniond(camera_to_robot_coordinate_frame_rotation);

    Eigen::Matrix3d camera_to_robot_rotation(camera_to_robot_rotation_);
    std::cout << camera_to_robot_rotation << std::endl;

    NODELET_INFO_STREAM("\n" << "=============================================\n"
                    << "Initialized Kalman Filter\n"
                    << "=============================================\n"
                    << "q = [" << qx_ << " , " << qy_ << " , " << qz_ << "]"
                    << ", r = [" << rx_ << " , " << ry_ << " , " << rz_ << "]" << "\n"
                    << "camera_to_robot_translation = [" << camera_to_robot_translation_.x() << " , " << camera_to_robot_translation_.y() << " , " << camera_to_robot_translation_.z() << "]\n"
                    << "camera_to_robot_rotation = [" << camera_to_robot_rotation_.w() << " , " << camera_to_robot_rotation_.x() << " , " << camera_to_robot_rotation_.y() << " , " << camera_to_robot_rotation_.z() << "]\n");

    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("kalman/output/cavity_pose", 1);

    robot_pose_sub_ = nh_.subscribe("kalman/input/robot_pose", 1, &KalmanNodelet::robotCb, this, ros::TransportHints().tcpNoDelay());
    cavity_pose_sub_ = nh_.subscribe("kalman/input/cavity_pose", 1,  &KalmanNodelet::cavityCb, this, ros::TransportHints().tcpNoDelay());

    reset_kalman_service_ = nh_.advertiseService("kalman/reset_kalman", &KalmanNodelet::resetKalmanCb, this);
    get_params_service_ = nh_.advertiseService("kalman/get_params", &KalmanNodelet::getParams, this);
    set_params_service_ = nh_.advertiseService("kalman/set_params", &KalmanNodelet::setParams, this);
}

void KalmanNodelet::updateStats()
{
    boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration elapsedTime = endTime - start_time_;
    if(elapsedTime.total_milliseconds() > 0)
        frequency_.push_back((int)1000/elapsedTime.total_milliseconds());

    // Maximum size of queue is 100
    if (frequency_.size() == 100)
    {
        int sum = std::accumulate(frequency_.begin(),frequency_.end(),0);
        avg_frequency_ = (int) sum / 100;
        frequency_.clear();
    }
}

bool KalmanNodelet::resetKalmanCb(rci_comm::ResetKalmanRequest &req, rci_comm::ResetKalmanResponse &res)
{
   got_first_cavity_measurement_ = false;
   measured_cavity_position_robot_ = Eigen::Vector3d::Zero();
   cavity_position_covariance_ = Eigen::Matrix3d::Identity();

    res.result = true;

    NODELET_DEBUG_STREAM("Reset Kalman request.");
    return true;
}

bool KalmanNodelet::getParams(rci_comm::GetKalmanParamsRequest &req, rci_comm::GetKalmanParamsResponse &res)
{
    if(fixed_kalman_q_)
    {
        res.qx = kalman_q_(0,0);
        res.qy = kalman_q_(1,1);
        res.qz = kalman_q_(2,2);
        res.fixed_q = true;
    }
    else
    {
        res.qx = 0;
        res.qy = 0;
        res.qz = 0;
        res.fixed_q = false;
    }
    res.rx = kalman_r_(0,0);
    res.ry = kalman_r_(1,1);
    res.rz = kalman_r_(2,2);

    return true;
}

bool KalmanNodelet::setParams(rci_comm::SetKalmanParamsRequest &req, rci_comm::SetKalmanParamsResponse &res)
{
    NODELET_DEBUG_STREAM_COND(req.fixed_q,"Set kalman params request with fixed q: q(" << req.qx << "," << req.qy << "," << req.qz << "), r(" << req.rx << "," << req.ry << "," << req.rz << ")");
    NODELET_DEBUG_STREAM_COND(!req.fixed_q,"Set kalman params request with not fixed q: q(" << req.qx << "," << req.qy << "," << req.qz << "), r(" << req.rx << "," << req.ry << "," << req.rz << ")");
    if(!req.fixed_q)
    {
        fixed_kalman_q_ = false;
        kalman_q_ = Eigen::Matrix3d::Zero();
    }
    else
    {
        fixed_kalman_q_ = true;
        if(req.qx != 1000 || req.qy != 1000 || req.qz != 1000)
        {
           kalman_q_(0,1) = 0;
           kalman_q_(0,2) = 0;
           kalman_q_(1,0) = 0;
           kalman_q_(1,2) = 0;
           kalman_q_(2,0) = 0;
           kalman_q_(2,1) = 0;
        }
        if(req.qx != 1000)
            kalman_q_(0,0) = req.qx;
        if(req.qy != 1000)
            kalman_q_(1,1) = req.qy;
        if(req.qz != 1000)
            kalman_q_(2,2) = req.qz;
    }
    if(req.rx != 1000)
        kalman_r_(0,0) = req.rx;
    if(req.ry != 1000)
        kalman_r_(1,1) = req.ry;
    if(req.rz != 1000)
        kalman_r_(2,2) = req.rz;

    res.result = true;
    return true;
}

void KalmanNodelet::cavityCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    //get measured cavity pose in camera frame
    Eigen::Vector3d measured_cavity_position_camera(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond measured_cavity_orientation_camera(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    if(msg->pose.position.x != 0 && msg->pose.position.y != 0 && msg->pose.position.z != 0)
    {
        //compute measured cavity pose in robot frame
        measured_cavity_position_robot_ = camera_to_robot_rotation_ * measured_cavity_position_camera;
        measured_cavity_position_robot_ = measured_cavity_position_robot_ + camera_to_robot_translation_;

        measured_cavity_orientation_robot_ = camera_to_robot_rotation_ * measured_cavity_orientation_camera;
    }
    else
    {
        measured_cavity_orientation_robot_ = Eigen::Quaterniond(0,0,0,0);
    }
}

void KalmanNodelet::robotCb(const nav_msgs::OdometryConstPtr &msg)
{
    KalmanNodelet::updateStats();
    start_time_ = boost::posix_time::microsec_clock::local_time();

    Eigen::Quaterniond new_robot_orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d new_robot_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    //compute transformation from world to robot frame k
    Eigen::Matrix3d robot_rotation_k(new_robot_orientation);
    Eigen::Vector3d robot_translation_k = new_robot_position;

    if(got_first_robot_pose_)
    {
        ++count_time_;
        boost::chrono::high_resolution_clock::time_point t_start_all = boost::chrono::high_resolution_clock::now();

        //compute transformation from robot frame k-1 to robot frame k
        Eigen::Matrix3d robot_rotation_kminus1_to_k = robot_rotation_k.transpose() * robot_rotation_kminus1_; //the inverse of a rotation matrix = transpose
        Eigen::Vector3d robot_translation_kminus1_to_k = robot_translation_kminus1_ - robot_translation_k;

        if(got_first_cavity_measurement_)
        {
            //Do Kalman filter predition update (compute estimated cavity pose in robot frame)
            estimated_cavity_position_robot_ = robot_rotation_kminus1_to_k * estimated_cavity_position_robot_ + robot_translation_kminus1_to_k;
            if(!fixed_kalman_q_) //if q=0 use the covariance (of the rotation) given by the robot pose
            {
                for(unsigned i = 0; i < 3; ++i)
                    for(unsigned j = 0; j < 3; ++j)
                        kalman_q_(i, j) = msg->pose.covariance[(i+3)*6 + j + 3]; //3 is the offset for rotaion cov, for position cov it would be 0
            }

            cavity_position_covariance_ = robot_rotation_kminus1_to_k * cavity_position_covariance_ * robot_rotation_kminus1_to_k.transpose() + kalman_q_;

            if(measured_cavity_position_robot_.x() != 0 && measured_cavity_position_robot_.y() != 0 && measured_cavity_position_robot_.z() != 0)
            {
                //Do Kalman filter correction update (compute estimated cavity pose in robot frame)
                Eigen::Matrix3d kalman_gain = cavity_position_covariance_ * (cavity_position_covariance_ + kalman_r_).inverse();
                estimated_cavity_position_robot_ = estimated_cavity_position_robot_ + kalman_gain * (measured_cavity_position_robot_ - estimated_cavity_position_robot_);
                cavity_position_covariance_ = (Eigen::Matrix3d::Identity() - kalman_gain) * cavity_position_covariance_
                                            * (Eigen::Matrix3d::Identity() - kalman_gain).transpose() + kalman_gain * kalman_r_ * kalman_gain.transpose(); //Joseph form
                measured_cavity_position_robot_ = Eigen::Vector3d::Zero();
            }

            geometry_msgs::PoseStamped cavity_pose;
            cavity_pose.header = msg->header;
            cavity_pose.header.frame_id = "robot";
            cavity_pose.pose.position.x = estimated_cavity_position_robot_.x();
            cavity_pose.pose.position.y = estimated_cavity_position_robot_.y();
            cavity_pose.pose.position.z = estimated_cavity_position_robot_.z();
            cavity_pose.pose.orientation.w = measured_cavity_orientation_robot_.w();
            cavity_pose.pose.orientation.x = measured_cavity_orientation_robot_.x();
            cavity_pose.pose.orientation.y = measured_cavity_orientation_robot_.y();
            cavity_pose.pose.orientation.z = measured_cavity_orientation_robot_.z();
            pub_.publish(cavity_pose);
        }
        else
        {
            if(measured_cavity_position_robot_.x() != 0 && measured_cavity_position_robot_.y() != 0 && measured_cavity_position_robot_.z() != 0)
            {
                //initialize Kalman filter with first measurement
                estimated_cavity_position_robot_ = measured_cavity_position_robot_;

                got_first_cavity_measurement_ = true;
                measured_cavity_position_robot_ = Eigen::Vector3d::Zero();

                geometry_msgs::PoseStamped cavity_pose;
                cavity_pose.header = msg->header;
                cavity_pose.header.frame_id = "robot";
                cavity_pose.pose.position.x = estimated_cavity_position_robot_.x();
                cavity_pose.pose.position.y = estimated_cavity_position_robot_.y();
                cavity_pose.pose.position.z = estimated_cavity_position_robot_.z();
                cavity_pose.pose.orientation.w = measured_cavity_orientation_robot_.w();
                cavity_pose.pose.orientation.x = measured_cavity_orientation_robot_.x();
                cavity_pose.pose.orientation.y = measured_cavity_orientation_robot_.y();
                cavity_pose.pose.orientation.z = measured_cavity_orientation_robot_.z();
                pub_.publish(cavity_pose);
            }
        }
        boost::chrono::high_resolution_clock::time_point t_end_all = boost::chrono::high_resolution_clock::now();
        time_all_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_all-t_start_all);

        if(count_time_ == 500)
        {
            boost::chrono::nanoseconds avg_time_all = time_all_ / count_time_;
            NODELET_DEBUG_STREAM("Time kalman : all=" << avg_time_all << " (" << (int)(1.0 / (avg_time_all.count()*1e-9)) << " Hz)");
            time_all_ = boost::chrono::nanoseconds(0);
            count_time_ = 0;
        }
    }
    else
    {
        got_first_robot_pose_ = true;
    }

    //update robot transformation at frame k-1
    robot_rotation_kminus1_ = robot_rotation_k;
    robot_translation_kminus1_ = robot_translation_k;

    NODELET_DEBUG_STREAM_THROTTLE(5,"The kalman runs at an avg of " << avg_frequency_ << " Hz");
}

} //namespace rci_kalman

PLUGINLIB_DECLARE_CLASS(rci_kalman, KalmanNodelet, rci_kalman::KalmanNodelet, nodelet::Nodelet);



