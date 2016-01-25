#include <inverse_kinematics_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rci_inverse_kinematics
{

InverseKinematicsNodelet::InverseKinematicsNodelet()
    : x1_(0),
      y1_(0),
      y2_(0),
      avg_frequency_(0),
      start_time_(boost::posix_time::microsec_clock::local_time())
{
}

void InverseKinematicsNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    nh_.param("is_simulated", is_simulated_, bool(false));

    nh_.param("arm/link_lengths/l1", l1_, double(0.24));
    nh_.param("arm/link_lengths/l3", l3_, double(0.35));
    nh_.param("arm/link_lengths/l5", l5_, double(0.28));
    nh_.param("arm/link_lengths/l6", l6_, double(0.12));
    nh_.param("arm/link_lengths/alpha", alpha_, double(2.3562));
    x2_ = l5_;

    nh_.param("arm/translation/x", robot_to_arm_translation_x_, double(0.0)); //to the front
    nh_.param("arm/translation/y", robot_to_arm_translation_y_, double(0.0));
    nh_.param("arm/translation/z", robot_to_arm_translation_z_, double(-0.03)); //up

    nh_.param("inverse_kinematics/n_semicircle", n_semicircle_, int(50));

    robot_to_arm_translation_ << robot_to_arm_translation_x_ , robot_to_arm_translation_y_ + (l5_/2) , robot_to_arm_translation_z_; //robot coords are in the middel, arm to the side -> +(l5/2)
//    Eigen::Matrix3d robot_to_arm_rotation;
//    robot_to_arm_rotation << 0 , 0 , 1,
//                            -1 , 0 , 0,
//                             0 , -1, 0;
//    robot_to_arm_rotation_ = Eigen::Quaterniond(robot_to_arm_rotation);

    semicircle_angles_.setLinSpaced(n_semicircle_,M_PI,2*M_PI); //setLinSpaced(size,low,high)


    NODELET_INFO_STREAM("\n" << "=============================================\n"
                    << "Initialized Inverse Kinematics\n"
                    << "=============================================\n"
    << "l1=l2=" << l1_ << " , l3=l4=" << l3_ << " , l5=" << l5_ << " , l6=" << l6_ << " , alpha=" << alpha_ << "\n"
    << "robot_to_arm_translation = [" << robot_to_arm_translation_.x() << " , " << robot_to_arm_translation_.y() << " , " << robot_to_arm_translation_.z() << "]\n"
    << "n_semicircle=" << n_semicircle_ << "\n");

    if(is_simulated_)
    {
        left_pub_ = nh_.advertise<manipulator_msgs::CommandPositionServoMotor>("inverse_kinematics/output/arm_goal_angle_left", 1);
        right_pub_ = nh_.advertise<manipulator_msgs::CommandPositionServoMotor>("inverse_kinematics/output/arm_goal_angle_right", 1);
    }
    else
    {
        joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("inverse_kinematics/output/joint_trajectory", 1);
    }

    endeffector_pose_sub_ = nh_.subscribe("inverse_kinematics/input/cavity_pose", 1,  &InverseKinematicsNodelet::endeffectorCb, this, ros::TransportHints().tcpNoDelay());
}

void InverseKinematicsNodelet::updateStats()
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

void InverseKinematicsNodelet::endeffectorCb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    //get desired endeffector pose in robot frame
    Eigen::Vector3d desired_endeffector_position_robot(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    //compute desired endeffector pose in arm frame
    //(z coord can be ignored since we consider arm in 2d and x & y are switched)
    Eigen::Vector2d desired_endeffector_position_arm(-desired_endeffector_position_robot.y() + robot_to_arm_translation_.y() , desired_endeffector_position_robot.x() + robot_to_arm_translation_.x());

    //std::cout << "robot frame: " << desired_endeffector_position_robot << "\narm frame: " << desired_endeffector_position_arm << std::endl;

    //find all possible (xp,yp)
    Eigen::VectorXd x_semicircle = (semicircle_angles_.array().cos() * l6_) + desired_endeffector_position_arm.x();
    Eigen::VectorXd y_semicircle = (semicircle_angles_.array().sin() * l6_) + desired_endeffector_position_arm.y();

    std::vector<double> results_q1(semicircle_angles_.size(),0.0);
    std::vector<double> results_q2(semicircle_angles_.size(),0.0);
    std::vector<double> results_error(semicircle_angles_.size(),1000.0);

    for(int i = 0; i < semicircle_angles_.size(); i++)
    {
        double xp_ref = x_semicircle(i);
        double yp_ref = y_semicircle(i);

        //try inverse kinematics with this xp & yp
        double a1 = l1_*l1_ + yp_ref*yp_ref + xp_ref*xp_ref - l3_*l3_ + 2*xp_ref*l1_;
        double a2 = l1_*l1_ + yp_ref*yp_ref + (xp_ref-l5_)*(xp_ref-l5_) - l3_*l3_ + 2*(xp_ref-l5_)*l1_;

        double b = -4*yp_ref*l1_;

        double c1 = l1_*l1_ + yp_ref*yp_ref + xp_ref*xp_ref - l3_*l3_ - 2*xp_ref*l1_;
        double c2 = l1_*l1_ + yp_ref*yp_ref + (xp_ref-l5_)*(xp_ref-l5_) - l3_*l3_ - 2*(xp_ref-l5_)*l1_;


        double z1 = (-b + sqrt(b*b - 4*a1*c1) ) / (2*a1);
        double z2 = (-b - sqrt(b*b - 4*a2*c2) ) / (2*a2);\

        double q1 = 2*atan(z1);
        double q2 = 2*atan(z2);

        // try forward kinematics with this q1 & q2
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

//        std::cout << "q1: " << q1 << " , calc q1: " << angle3Points(x3-x1_,y3-y1_,std::abs(x3-x1_),std::abs(y1_-y1_)) << std::endl;
//        std::cout << "q2: " << q2 << " , calc q2: " << angle3Points(x4-x2_,y4-y2_,std::abs(x4-x2_),std::abs(y2_-y2_)) << std::endl;
//        std::cout << "alpha: " << alpha_ << " , calc alpha: " << angle3Points(x-xp,y-yp,x4-xp,y4-yp) << std::endl;

        if (std::abs(dist2Points(x3,y3,x1_,y1_) - dist2Points(x4,y4,x2_,y2_)) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! L1 != L2");
        else if (std::abs(dist2Points(xp, yp, x3, y3) - dist2Points(xp,yp,x4,y4)) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! L3 != L4");
        else if (std::abs(dist2Points(x3,y3,x1_,y1_) - l1_) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! L1 is not correct!");
        else if (std::abs(dist2Points(x4,y4,x2_,y2_) - l1_) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! L2 is not correct!");
        else if (std::abs(dist2Points(xp,yp,x3,y3) - l3_) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! L3 is not correct!");
        else if (std::abs(dist2Points(xp,yp,x4,y4) - l3_) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! L4 is not correct!");
        else if (std::abs(dist2Points(x,y,xp,yp) - l6_) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! L6 is not correct!");
        else if (std::abs(angle3Points(x3-x1_,y3-y1_,std::abs(x3-x1_),std::abs(y1_-y1_)) - q1) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! q1 is not correct!");
        else if (std::abs(angle3Points(x4-x2_,y4-y2_,std::abs(x4-x2_),std::abs(y2_-y2_)) - q2) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! q2 is not correct!");
        else if (std::abs(angle3Points(x-xp,y-yp,x4-xp,y4-yp) - alpha_) > 0.0000000000001)
            NODELET_DEBUG_STREAM("Error! alpha is not correct!");
        else if (q1 != q1 || q2 != q2)
            NODELET_DEBUG_STREAM("Error! angle is not a number!");
        else
        {
            results_q1[i] = q1;
            results_q2[i] = q2;
            results_error[i] = dist2Points(desired_endeffector_position_arm.x(),desired_endeffector_position_arm.y(),x,y);
//            std::cout << "q1: " << q1 << " , q2: " << q2 << " , err: " << results_error[i] << std::endl;
        }
    }
    //Find min error result
    int min_idx = std::min_element(results_error.begin(), results_error.end()) - results_error.begin();

    if (results_error[min_idx] == 1000.0)
    {
        NODELET_ERROR_STREAM("!!!!!!!!!! No Possible Solution Found !!!!!!!!!!");
    }
    else
    {
        //publish final result
        double q1 = results_q1[min_idx];
        double q2 = results_q2[min_idx];
        std::cout << "q1: " << q1 << " , q2: " << q2 << ", err:" << results_error[min_idx] << std::endl;

        //different angle convention for real robot arm!
        if(!is_simulated_)
        {
            q1 = -1*(M_PI - q1);
        }

        if(is_simulated_)
        {
            if(q2 > 0.8)
                q2 = 0.8;
            if(q2 < 0.0)
                q2 = 0.0;
            if(q1 > 2.34)
                q1 = 2.34;
            if(q1 < 0.0)
                q1 = 0.0;

            std::cout << "q1: " << q1 << " , q2: " << q2 << std::endl;
            manipulator_msgs::CommandPositionServoMotor left_angle_output;
            left_angle_output.header.stamp = ros::Time::now();
            left_angle_output.motor_angle = q1;
            manipulator_msgs::CommandPositionServoMotor right_angle_output;
            right_angle_output.header.stamp = ros::Time::now();
            right_angle_output.motor_angle = q2;
            left_pub_.publish(left_angle_output);
            right_pub_.publish(right_angle_output);
        }
        else
        {
            if(q2 > 0.8)
                q2 = 0.8;
            if(q2 < 0.0)
                q2 = 0.0;
            if(q1 > 0.0)
                q1 = 0.0;
            if(q1 < -0.8)
                q1 = 0.8;

            trajectory_msgs::JointTrajectory output;
            trajectory_msgs::JointTrajectoryPoint joint_right, joint_left;
            output.header.stamp = ros::Time::now();
            output.joint_names = {"joint_1", "joint_2"};
            joint_right.positions.push_back(q2);
            joint_left.positions.push_back(q1);//different angle convention for real robot arm!
            output.points.push_back(joint_right);
            output.points.push_back(joint_left);
            joint_trajectory_pub_.publish(output);
        }
    }
}

double InverseKinematicsNodelet::dist2Points(double x1, double y1, double x2, double y2)
{
    double dist = sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
    return dist;
}

double InverseKinematicsNodelet::cross(double x1, double y1, double x2, double y2)
{
    double prod = x1*y2 - y1*x2;
    return prod;
}

double InverseKinematicsNodelet::dot(double x1, double y1, double x2, double y2)
{
    double prod = x1*x2 + y1*y2;
    return prod;
}

double InverseKinematicsNodelet::angle3Points(double ba_x, double ba_y, double bc_x, double bc_y)
{
    double angle = atan2(cross(ba_x, ba_y , bc_x ,bc_y),dot(ba_x, ba_y , bc_x ,bc_y));
    return std::abs(angle);
}

} //namespace rci_inverse_kinematics

PLUGINLIB_DECLARE_CLASS(rci_inverse_kinematics, InverseKinematicsNodelet, rci_inverse_kinematics::InverseKinematicsNodelet, nodelet::Nodelet);



