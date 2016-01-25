#include <controller_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rci_controller
{

ControllerNodelet::ControllerNodelet()
    : do_control_(false),
      received_first_cavity_position_(false),
      prev_arm_command_(Eigen::Vector3d::Zero()),
      avg_frequency_(0),
      start_time_(boost::posix_time::microsec_clock::local_time())
{
}

void ControllerNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    nh_.param("is_simulated", is_simulated_, bool(false));

    nh_.param("controller/desired_pitching_angle", desired_pitching_angle_, double(0.0));
    nh_.param("controller/robot_dist_to_tree", robot_dist_to_tree_, double(0.5));
    nh_.param("controller/cavity_depth", cavity_depth_, double(0.05));
    nh_.param("controller/tolerance/arm_pitching", arm_pitching_tol_, double(0.01));
    nh_.param("controller/tolerance/robot_position/x", robot_position_tol_x_, double(0.01));
    nh_.param("controller/tolerance/robot_position/y", robot_position_tol_y_, double(0.01));
    nh_.param("controller/tolerance/robot_position/z", robot_position_tol_z_, double(0.01));
    nh_.param("controller/tolerance/robot_orientation", robot_orientation_tol_, double(0.001));
    nh_.param("controller/filter_alpha", filter_alpha_, double(0.1));
    nh_.param("controller/update_rate", update_rate_, int(100));
    nh_.param("controller/robot_offset/x", robot_offset_x_, double(0.0));
    nh_.param("controller/robot_offset/y", robot_offset_y_, double(0.0));
    nh_.param("controller/robot_offset/z", robot_offset_z_, double(0.0));

    nh_.param("arm/translation/x", robot_to_arm_translation_x_, double(0.0));
    nh_.param("arm/translation/y", robot_to_arm_translation_y_, double(0.0));
    nh_.param("arm/translation/z", robot_to_arm_translation_z_, double(-0.03));
    nh_.param("arm/link_lengths/l1", l1_, double(0.24));
    nh_.param("arm/link_lengths/l3", l3_, double(0.35));
    nh_.param("arm/link_lengths/l5", l5_, double(0.28));
    nh_.param("arm/link_lengths/l6", l6_, double(0.12));
    nh_.param("arm/link_lengths/alpha", alpha_, double(2.3562));
    x2_ = l5_;

    std::cout << "pitching dist=" << tan(desired_pitching_angle_)*robot_dist_to_tree_ << std::endl;

    robot_to_arm_translation_ << robot_to_arm_translation_x_ , robot_to_arm_translation_y_ , robot_to_arm_translation_z_;
    robot_offset_ << robot_offset_x_ , robot_offset_y_ , robot_offset_z_;

    NODELET_INFO_STREAM("\n" << "=============================================\n"
                    << "Initialized Controller\n"
                    << "=============================================\n"
                    << "desired_pitching_angle = " << desired_pitching_angle_ << " [rad]"
                    << ", robot_dist_to_tree = " << robot_dist_to_tree_ << " [m]"
                    << ", cavity_depth = " << cavity_depth_ << " [m]" << "\n"
                    << "arm_pitching_tol = " << arm_pitching_tol_ << " [m]"
                    << ", robot_orientation_tol = " << robot_orientation_tol_ << " [m]" << "\n"
                    << "robot_position_tol = [" << robot_position_tol_x_ << " , " << robot_position_tol_y_ << " , " << robot_position_tol_z_ << "]\n"
                    << "l1=l2=" << l1_ << " , l3=l4=" << l3_ << " , l5=" << l5_ << " , l6=" << l6_ << " , alpha=" << alpha_ << "\n"
                    << "arm_to_robot_translation = [" << robot_to_arm_translation_.x() << " , " << robot_to_arm_translation_.y() << " , " << robot_to_arm_translation_.z() << "]\n"
                    << "robot_offset = [" << robot_offset_.x() << " , " << robot_offset_.y() << " , " << robot_offset_.z() << "]\n"
                    << "filter_alpha = " << filter_alpha_ << "\n");

    arm_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("controller/output/arm_goal_pose", 1);
    robot_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("controller/output/robot_goal_pose", 1);
    if (is_simulated_)
    {
        pitching_pub_ = nh_.advertise<manipulator_msgs::CommandPositionServoMotor>("controller/output/angle_pitching", 1);
        left_pub_ = nh_.advertise<manipulator_msgs::CommandPositionServoMotor>("controller/output/angle_left", 1);
        right_pub_ = nh_.advertise<manipulator_msgs::CommandPositionServoMotor>("controller/output/angle_right", 1);
    }
    else
    {
        joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("controller/output/joint_trajectory", 1);
    }
    robot_goal_debug_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("controller/output/robot_goal_debug_pose", 1);

    robot_pose_sub_ = nh_.subscribe("controller/input/robot_pose", 1, &ControllerNodelet::robotCb, this, ros::TransportHints().tcpNoDelay());
    cavity_pose_sub_ = nh_.subscribe("controller/input/cavity_pose", 1,  &ControllerNodelet::cavityCb, this, ros::TransportHints().tcpNoDelay());
    pitching_jointstate_sub_ = nh_.subscribe("controller/input/jointstate/pitching", 1,  &ControllerNodelet::jointstatePitchingCb, this, ros::TransportHints().tcpNoDelay());
    right_jointstate_sub_ = nh_.subscribe("controller/input/jointstate/right", 1,  &ControllerNodelet::jointstateRightCb, this, ros::TransportHints().tcpNoDelay());
    left_jointstate_sub_ = nh_.subscribe("controller/input/jointstate/left", 1,  &ControllerNodelet::jointstateLeftCb, this, ros::TransportHints().tcpNoDelay());

    start_controller_service_ = nh_.advertiseService("controller/start_controller", &ControllerNodelet::startControllerCb, this);
    lift_arm_service_ = nh_.advertiseService("controller/lift_arm", &ControllerNodelet::liftArmCb, this);
    get_params_service_ = nh_.advertiseService("controller/get_params", &ControllerNodelet::getParams, this);
    set_params_service_ = nh_.advertiseService("controller/set_params", &ControllerNodelet::setParams, this);

    update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &ControllerNodelet::updateCb, this);



    Eigen::Vector3d cavity_normal(-1, -0.3, 0.0  );
    Eigen::Quaterniond robot_or(0.954,-0.0060,-0.0050,-0.2970);
    std::cout << robot_or.w() << std::endl;

    Eigen::Quaterniond new_robot_orientation;
    Eigen::Quaterniond robot_rotation;
    robot_rotation.setFromTwoVectors(Eigen::Vector3d::UnitX(),-1*cavity_normal);
    Eigen::Quaterniond new_robot_orientation_temp = robot_rotation * robot_or;
    new_robot_orientation = new_robot_orientation_temp;
    NODELET_WARN_STREAM("new_robot_orientation(" << new_robot_orientation.w() << "," << new_robot_orientation.x() << "," << new_robot_orientation.y() << "," << new_robot_orientation.z() << ")");
}

void ControllerNodelet::updateStats()
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

bool ControllerNodelet::startControllerCb(rci_comm::StartControllerRequest &req, rci_comm::StartControllerResponse &res)
{
    if(!req.start)
    {
        do_control_ = false;
        res.result = 0; //success
        NODELET_INFO_STREAM("Stop Controller request.");
    }
    else if(!received_first_cavity_position_)
    {
        do_control_ = false;
        res.result = 1; //failure because no cavity
        NODELET_INFO_STREAM("Start Controller request. No Cavity!");
    }
//    else if(std::abs(pitching_angle_ - desired_pitching_angle_) > arm_pitching_tol_)
//    {
//        do_control_ = false;
//        res.result = 2; //failure because arm not lifted
//        NODELET_INFO_STREAM("Start Controller request. Arm not lifted! Error: " << std::abs(pitching_angle_ - desired_pitching_angle_) << ", current: " << pitching_angle_ << ", reference: " << desired_pitching_angle_);
//    }
    else
    {
        do_control_ = true;
        res.result = 0; // success
        NODELET_INFO_STREAM("Start Controller request.");
    }

    return true;
}

bool ControllerNodelet::liftArmCb(rci_comm::LiftArmRequest &req, rci_comm::LiftArmResponse &res)
{
    if(is_simulated_)
    {
        pitching_pub_.publish(createCommandPositionServoMotorMsg(desired_pitching_angle_));
        left_pub_.publish(createCommandPositionServoMotorMsg(2.7489));
        right_pub_.publish(createCommandPositionServoMotorMsg(0.3927));
    }
    else
    {
        trajectory_msgs::JointTrajectory output;
        trajectory_msgs::JointTrajectoryPoint joint_pitching, joint_right, joint_left;
        output.header.stamp = ros::Time::now();
        output.joint_names = {"joint_0", "joint_1", "joint_2"};
        joint_pitching.positions.push_back(desired_pitching_angle_);
        joint_right.positions.push_back(0.3927);
        joint_left.positions.push_back(-0.3927);//different angle convention for real robot arm!
        output.points.push_back(joint_pitching);
        output.points.push_back(joint_right);
        output.points.push_back(joint_left);
        joint_trajectory_pub_.publish(output);
    }

    res.result = true;

    NODELET_INFO_STREAM("Lift Arm request.");
    return true;
}

bool ControllerNodelet::getParams(rci_comm::GetControllerParamsRequest &req, rci_comm::GetControllerParamsResponse &res)
{
    res.desired_pitching_angle = desired_pitching_angle_;
    res.robot_dist_to_tree = robot_dist_to_tree_;
    res.arm_pitching_tol = arm_pitching_tol_;
    res.robot_position_tol_x = robot_position_tol_x_;
    res.robot_position_tol_y = robot_position_tol_y_;
    res.robot_position_tol_z = robot_position_tol_z_;
    res.robot_orientation_tol = robot_orientation_tol_;
    res.robot_offset_x = robot_offset_.x();
    res.robot_offset_y = robot_offset_.y();
    res.robot_offset_z = robot_offset_.z();

    return true;
}

bool ControllerNodelet::setParams(rci_comm::SetControllerParamsRequest &req, rci_comm::SetControllerParamsResponse &res)
{
    NODELET_INFO_STREAM("Set controller params request");
    if(req.desired_pitching_angle != 1000)
        desired_pitching_angle_ = req.desired_pitching_angle;
    if(req.robot_dist_to_tree != 1000)
        robot_dist_to_tree_ = req.robot_dist_to_tree;
    if(req.arm_pitching_tol != 1000)
        arm_pitching_tol_ = req.arm_pitching_tol;
    if(req.robot_position_tol_x != 1000)
        robot_position_tol_x_ = req.robot_position_tol_x;
    if(req.robot_position_tol_y != 1000)
        robot_position_tol_y_ = req.robot_position_tol_y;
    if(req.robot_position_tol_z != 1000)
        robot_position_tol_z_ = req.robot_position_tol_z;
    if(req.robot_orientation_tol != 1000)
        robot_orientation_tol_ = req.robot_orientation_tol;
    if(req.robot_offset_x != 1000)
        robot_offset_.x() = req.robot_offset_x;
    if(req.robot_offset_y != 1000)
        robot_offset_.y() = req.robot_offset_y;
    if(req.robot_offset_z != 1000)
        robot_offset_.z() = req.robot_offset_z;

    res.result = true;
    return true;
}

void ControllerNodelet::cavityCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if(!(msg->pose.position.x == 0 && msg->pose.position.y == 0 && msg->pose.position.z == 0))
    {
        received_first_cavity_position_ = true;
        cavity_orientation_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        cavity_position_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }
}

void ControllerNodelet::robotCb(const nav_msgs::OdometryConstPtr &msg)
{
    robot_orientation_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    robot_position_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void ControllerNodelet::jointstatePitchingCb(const sensor_msgs::JointStateConstPtr &msg)
{
    pitching_angle_ = msg->position[0];
}

void ControllerNodelet::jointstateRightCb(const sensor_msgs::JointStateConstPtr &msg)
{
    right_angle_ = msg->position[0];
}

void ControllerNodelet::jointstateLeftCb(const sensor_msgs::JointStateConstPtr &msg)
{
    left_angle_ = msg->position[0];
}

Eigen::Vector3d ControllerNodelet::forwardKinematics(double q1, double q2)
{
    //different angle convention for real robot arm!
    if(!is_simulated_)
    {
        q1 = M_PI + q1;
    }

    // do forward kinematics with this q1 & q2
    double a3 = 2*l3_*(l1_*cos(q1) - l1_*cos(q2) - l5_);
    double b3 = 2*l3_*(l1_*sin(q1) - l1_*sin(q2));
    double c3 = -2*l1_*l1_ - l5_*l5_ + 2*l1_*l1_*sin(q1)*sin(q2) + 2*l1_*cos(q1)*(l1_*cos(q2) + l5_) - 2*l1_*l5_*cos(q2);
    double q3 = 2*(atan((b3 - sqrt(a3*a3 + b3*b3 - c3*c3))/(a3+c3)));
    double q4 = M_PI - asin((l1_*sin(q1) + l3_*sin(q3) - l1_*sin(q2)) / (l3_));

    double x3 = l1_*cos(q1);
    double y3 = l1_*sin(q1);
    double x4 = l5_ + l1_*cos(q2);
    double y4 = l1_*sin(q2);
    double xp = x3 + l3_*cos(q3);
    double yp = y3 + l3_*sin(q3);
    double angle = alpha_ - M_PI + q4;
    double x = xp + l6_*cos(angle);
    double y = yp + l6_*sin(angle);

    //std::cout << "end in arm frame: " << x << " , " << y << std::endl;
    //(z coord can be ignored since we consider arm in 2d and x & y are switched)
    Eigen::Vector3d endeffector_position(y-robot_to_arm_translation_x_, (-1)*x + robot_to_arm_translation_y_ + (l5_/2), 0.0);
    return endeffector_position;
}

void ControllerNodelet::updateCb(const ros::TimerEvent &e)
{
    //if service of autopilot has been activated and received first cavity position and orientation
    if(do_control_ && received_first_cavity_position_)
    {
        desired_cavity_position_ << robot_dist_to_tree_ , robot_to_arm_translation_y_ ,
                                    robot_to_arm_translation_z_+tan(desired_pitching_angle_)*robot_dist_to_tree_;

//        std::cout << std::endl;
//        std::cout << "cavity_position: " << cavity_position_.x() << "," << cavity_position_.y() << "," << cavity_position_.z()
//                  << ", desired_cavity: " << desired_cavity_position_.x() << "," << desired_cavity_position_.y() << "," << desired_cavity_position_.z()
//                  << ", dist: (" << std::abs(cavity_position_.x() - desired_cavity_position_.x()) << "," << std::abs(cavity_position_.y() - desired_cavity_position_.y()) << "," << std::abs(cavity_position_.z() - desired_cavity_position_.z()) << std::endl;
//        Eigen::Vector3d cavity_position_world = robot_orientation_.inverse() * cavity_position_;
//        cavity_position_world += robot_position_;
//        std::cout << "cavity_position_world: " << cavity_position_world.x() << "," << cavity_position_world.y() << "," << cavity_position_world.z() << std::endl;

//        std::cout << "robot position : " << robot_position_.x() << "," << robot_position_.y() << "," << robot_position_.z() << std::endl;
//        std::cout << "robot: orientation(" << robot_orientation_.w() << "," << robot_orientation_.x() << "," << robot_orientation_.y() << "," << robot_orientation_.z() << ")" << std::endl;

//        std::cout << "cavity orientation(" << cavity_orientation_.w() << "," << cavity_orientation_.x() << "," << cavity_orientation_.y() << "," << cavity_orientation_.z() << ")" << std::endl;

        //check if robot position is correctly hovering in front of cavity,
        //i.e cavity position in robot frame should be (robot_dist_to_tree, arm_tanslation_y, arm_translation_z) = desired_cavity_position

        //move the robot to hover 50cm in front of the cavity (and displaced in y & z direction according to mounting of arm)
        Eigen::Vector3d new_robot_position = robot_position_ + (cavity_position_ - desired_cavity_position_);
        NODELET_DEBUG_STREAM("new robot position : " << new_robot_position.x() << " (" << std::abs(cavity_position_.x() - desired_cavity_position_.x()) << ") ,"
                            << new_robot_position.y() << " (" << std::abs(cavity_position_.y() - desired_cavity_position_.y()) << ") ,"
                            << new_robot_position.z() << " (" << std::abs(cavity_position_.z() - desired_cavity_position_.z()) << ")" );

        Eigen::Quaterniond new_robot_orientation = robot_orientation_;
        if(cavity_orientation_.x() != 0 && cavity_orientation_.y() != 0 && cavity_orientation_.z() != 0 && cavity_orientation_.w() != 0)
        {
            Eigen::Matrix3d cavity_orientation_rotm(cavity_orientation_);
            Eigen::Vector3d cavity_normal = cavity_orientation_rotm.col(2);
            //std::cout << "cavity normal(" << cavity_normal.x() << "," << cavity_normal.y() << "," << cavity_normal.z() << ")" << std::endl;

            Eigen::Quaterniond robot_rotation;
            robot_rotation.setFromTwoVectors(Eigen::Vector3d::UnitX(),-1*cavity_normal);
            Eigen::Quaterniond new_robot_orientation_temp = robot_rotation * robot_orientation_;
            new_robot_orientation = new_robot_orientation_temp;
            NODELET_DEBUG_STREAM("new_robot_orientation(" << new_robot_orientation.w() << "," << new_robot_orientation.x() << "," << new_robot_orientation.y() << "," << new_robot_orientation.z() << ")");
        }

        robot_goal_debug_pub_.publish(createPoseStampedMsgFromEigen(new_robot_position,new_robot_orientation,"world"));
        new_robot_position = new_robot_position + robot_offset_;
        //std::cout << "new robot position with offset: " << new_robot_position.x() << "," << new_robot_position.y() << "," << new_robot_position.z() << std::endl;
        robot_goal_pub_.publish(createPoseStampedMsgFromEigen(new_robot_position,new_robot_orientation,"world"));

        if(std::abs(cavity_position_.x() - desired_cavity_position_.x()) < robot_position_tol_x_)
        {
            //std::cout << "cavity_position: " << cavity_position_.x() << "," << cavity_position_.y() << "," << cavity_position_.z() << std::endl;
            Eigen::Vector3d endeffector_position = forwardKinematics(left_angle_, right_angle_);
            //std::cout << "endeffector_position: " << endeffector_position.x() << "," << endeffector_position.y() << "," << endeffector_position.z() << std::endl;
            Eigen::Vector3d target_position(cavity_position_.x()+ cavity_depth_, cavity_position_.y(), 0);

            NODELET_DEBUG_STREAM("new arm position! move inside cavity entrance (" << target_position.x() << "," << target_position.y() << "," << target_position.z() << ")");

            //first order filter
            if(prev_arm_command_.x() == 0 && prev_arm_command_.y() == 0 && prev_arm_command_.z() == 0)
            {
                //initilize prev_arm_command_ to current eneffector position
                prev_arm_command_ = endeffector_position;
            }
            Eigen::Vector3d new_output = filter_alpha_ * target_position + (1-filter_alpha_) * prev_arm_command_;
            prev_arm_command_ = new_output;
            arm_goal_pub_.publish(createPoseStampedMsgFromEigen(new_output, Eigen::Quaterniond(0,0,0,0),"robot"));
        }

/*
        if(std::abs(cavity_position_.x() - desired_cavity_position_.x()) > robot_position_tol_x_ || std::abs(cavity_position_.y() - desired_cavity_position_.y()) > robot_position_tol_y_ || std::abs(cavity_position_.z() - desired_cavity_position_.z()) > robot_position_tol_z_)
        {
            //move the robot to hover 50cm in front of the cavity (and displaced in y & z direction according to mounting of arm)
            Eigen::Vector3d new_robot_position = robot_position_ + (cavity_position_ - desired_cavity_position_);
            NODELET_INFO_STREAM("new robot position : " << new_robot_position.x() << " (" << std::abs(cavity_position_.x() - desired_cavity_position_.x()) << ") ,"
                                << new_robot_position.y() << " (" << std::abs(cavity_position_.y() - desired_cavity_position_.y()) << ") ,"
                                << new_robot_position.z() << " (" << std::abs(cavity_position_.z() - desired_cavity_position_.z()) << ")" );

            Eigen::Quaterniond new_robot_orientation = robot_orientation_;
            if(cavity_orientation_.x() != 0 && cavity_orientation_.y() != 0 && cavity_orientation_.z() != 0 && cavity_orientation_.w() != 0)
            {
                Eigen::Matrix3d cavity_orientation_rotm(cavity_orientation_);
                Eigen::Vector3d cavity_normal = cavity_orientation_rotm.col(2);
                //std::cout << "cavity normal(" << cavity_normal.x() << "," << cavity_normal.y() << "," << cavity_normal.z() << ")" << std::endl;

                Eigen::Quaterniond robot_rotation;
                robot_rotation.setFromTwoVectors(Eigen::Vector3d::UnitX(),-1*cavity_normal);
                Eigen::Quaterniond new_robot_orientation_temp = robot_rotation * robot_orientation_;
                if(!new_robot_orientation_temp.isApprox(robot_orientation_,robot_orientation_tol_)) //check if robot orientation is close enough aligned to cavity orientation
                {
                    new_robot_orientation = new_robot_orientation_temp;
                    NODELET_INFO_STREAM("new_robot_orientation(" << new_robot_orientation.w() << "," << new_robot_orientation.x() << "," << new_robot_orientation.y() << "," << new_robot_orientation.z() << ")");
                }
            }

            robot_goal_debug_pub_.publish(createPoseStampedMsgFromEigen(new_robot_position,new_robot_orientation,"world"));

            new_robot_position = new_robot_position + robot_offset_;
            //std::cout << "new robot position with offset: " << new_robot_position.x() << "," << new_robot_position.y() << "," << new_robot_position.z() << std::endl;

            robot_goal_pub_.publish(createPoseStampedMsgFromEigen(new_robot_position,new_robot_orientation,"world"));
        }
        else
        {
            Eigen::Matrix3d cavity_orientation_rotm(cavity_orientation_);
            Eigen::Vector3d cavity_normal = cavity_orientation_rotm.col(2);
            Eigen::Quaterniond robot_rotation;
            robot_rotation.setFromTwoVectors(Eigen::Vector3d::UnitX(),-1*cavity_normal);
            Eigen::Quaterniond new_robot_orientation = robot_rotation * robot_orientation_;
            //std::cout << "cavity normal(" << cavity_normal.x() << "," << cavity_normal.y() << "," << cavity_normal.z() << ")" << std::endl;

            if(cavity_orientation_.x() != 0 && cavity_orientation_.y() != 0 && cavity_orientation_.z() != 0 && cavity_orientation_.w() != 0
                    && !new_robot_orientation.isApprox(robot_orientation_,robot_orientation_tol_)) //check if robot orientation is close enough aligned to cavity orientation
            {
                robot_goal_pub_.publish(createPoseStampedMsgFromEigen(robot_position_ + robot_offset_, new_robot_orientation, "world"));
                NODELET_WARN_STREAM("new_robot_orientation(" << new_robot_orientation.w() << "," << new_robot_orientation.x() << "," << new_robot_orientation.y() << "," << new_robot_orientation.z() << ")");

            }
            else
            {
//                std::cout << "cavity_position: " << cavity_position_.x() << "," << cavity_position_.y() << "," << cavity_position_.z() << std::endl;
                Eigen::Vector3d endeffector_position = forwardKinematics(left_angle_, right_angle_);
//                std::cout << "endeffector_position: " << endeffector_position.x() << "," << endeffector_position.y() << "," << endeffector_position.z() << std::endl;
                Eigen::Vector3d target_position(cavity_position_.x(), cavity_position_.y(), 0.0);
                if(target_position.x() - endeffector_position.x() > robot_position_tol_x_)
                {
                    NODELET_ERROR_STREAM("new arm position! move to cavity entrance");
//                    Eigen::Vector3d trajectory_vector = target_position - endeffector_position;
//                    trajectory_vector.normalize();
//                    trajectory_vector = 0.01 * trajectory_vector;
//                    std::cout << "trajectory position : " << endeffector_position + trajectory_vector << std::endl;

                    //first order filter
                    if(prev_arm_command_.x() == 0 && prev_arm_command_.y() == 0 && prev_arm_command_.z() == 0)
                    {
                        //initilize prev_arm_command_ to current eneffector position
                        prev_arm_command_ = endeffector_position;
                    }
                    Eigen::Vector3d new_output = filter_alpha_ * target_position + (1-filter_alpha_) * prev_arm_command_;
                    prev_arm_command_ = new_output;
                    arm_goal_pub_.publish(createPoseStampedMsgFromEigen(new_output, Eigen::Quaterniond(0,0,0,0),"robot"));
                    //arm_goal_pub_.publish(createPoseStampedMsgFromEigen(endeffector_position + trajectory_vector , Eigen::Quaterniond(0,0,0,0),"robot"));
                }
                else
                {
                    NODELET_ERROR_STREAM("new arm position! move INSIDE cavity");
                    target_position.x() += cavity_depth_;
                    //std::cout << "target position : " << target_position.x() << "," << target_position.y() << std::endl;
                    if(std::abs(endeffector_position.x() - target_position.x()) >= 0.01 || std::abs(endeffector_position.y() - target_position.y()) >= 0.01)
                    {
//                        Eigen::Vector3d trajectory_vector = target_position - endeffector_position;
//                        trajectory_vector.normalize();
//                        trajectory_vector = 0.01 * trajectory_vector;
//                        std::cout << "trajectory position : " << endeffector_position + trajectory_vector << std::endl;
//                        arm_goal_pub_.publish(createPoseStampedMsgFromEigen(endeffector_position + trajectory_vector , Eigen::Quaterniond(0,0,0,0),"robot"));
                        //first order filter
                        if(prev_arm_command_.x() == 0 && prev_arm_command_.y() == 0 && prev_arm_command_.z() == 0)
                        {
                            //initilize prev_arm_command_ to current eneffector position
                            prev_arm_command_ = endeffector_position;
                        }
                        Eigen::Vector3d new_output = filter_alpha_ * target_position + (1-filter_alpha_) * prev_arm_command_;
                        prev_arm_command_ = new_output;
                        arm_goal_pub_.publish(createPoseStampedMsgFromEigen(new_output, Eigen::Quaterniond(0,0,0,0),"robot"));
                    }
                }
            }
        }
*/
    }
}

manipulator_msgs::CommandPositionServoMotor ControllerNodelet::createCommandPositionServoMotorMsg(double position)
{
    manipulator_msgs::CommandPositionServoMotor position_output;
    position_output.header.stamp = ros::Time::now();
    position_output.motor_angle = position;
    return position_output;
}

geometry_msgs::PoseStamped ControllerNodelet::createPoseStampedMsgFromEigen(Eigen::Vector3d position, Eigen::Quaterniond orientation, std::string id)
{
    geometry_msgs::PoseStamped output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = id;
    output.pose.position.x = position.x();
    output.pose.position.y = position.y();
    output.pose.position.z = position.z();
    output.pose.orientation.w = orientation.w();
    output.pose.orientation.x = orientation.x();
    output.pose.orientation.y = orientation.y();
    output.pose.orientation.z = orientation.z();
    return output;
}

} //namespace rci_controller

PLUGINLIB_DECLARE_CLASS(rci_controller, ControllerNodelet, rci_controller::ControllerNodelet, nodelet::Nodelet);
