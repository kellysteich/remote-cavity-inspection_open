#include <cavity_normal_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rci_cavity_normal
{

CavityNormalNodelet::CavityNormalNodelet()
    : avg_frequency_(0),
      start_time_(boost::posix_time::microsec_clock::local_time())
{
}

void CavityNormalNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    nh_.param("cavity_normal/band_width", band_width_, double(0.01));
    nh_.param("box_check/octree/resolution", octree_resolution_, double(0.001));

    NODELET_INFO_STREAM("\n" << "=============================================\n"
                    << "Initialized Cavity Normal Detection\n"
                    << "=============================================\n"
                    << "band_width = " << band_width_ << " [m]\n");

    pub_ = nh_.advertise<rci_comm::CavityBoxNormal>("cavity_normal/output/cavity_box", 1);
    cavity_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cavity_normal/output/cavity_pose", 1);

    cloud_sub_.subscribe(nh_, "cavity_normal/input/points", 4);
    cavity_sub_.subscribe(nh_, "cavity_normal/input/cavity", 4);
    sync_ = boost::make_shared<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, rci_comm::CavityBox> > (8);
    sync_->connectInput (cloud_sub_, cavity_sub_);
    sync_->registerCallback (bind (&CavityNormalNodelet::cb, this, _1, _2));

    get_params_service_ = nh_.advertiseService("cavity_normal/get_params", &CavityNormalNodelet::getParams, this);
    set_params_service_ = nh_.advertiseService("cavity_normal/set_params", &CavityNormalNodelet::setParams, this);
}

void CavityNormalNodelet::updateStats()
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

bool CavityNormalNodelet::getParams(rci_comm::GetCavityNormalParamsRequest &req, rci_comm::GetCavityNormalParamsResponse &res)
{
    res.band_width = band_width_;

    return true;
}

bool CavityNormalNodelet::setParams(rci_comm::SetCavityNormalParamsRequest &req, rci_comm::SetCavityNormalParamsResponse &res)
{
    NODELET_DEBUG_STREAM("Set cavity normal params request: band_width:" << req.band_width);
    if(req.band_width != 1000)
        band_width_ = req.band_width;

    res.result = true;
    return true;
}

void CavityNormalNodelet::cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const rci_comm::CavityBoxConstPtr &cavity_msg)
{
    CavityNormalNodelet::updateStats();
    start_time_ = boost::posix_time::microsec_clock::local_time();

    rci_comm::CavityBoxNormal cavity_normal;
    cavity_normal.header = cavity_msg->header;
    cavity_normal.min_x = cavity_msg->min_x;
    cavity_normal.min_y = cavity_msg->min_y;
    cavity_normal.min_z = cavity_msg->min_z;
    cavity_normal.max_x = cavity_msg->max_x;
    cavity_normal.max_y = cavity_msg->max_y;
    cavity_normal.max_z = cavity_msg->max_z;

    geometry_msgs::PoseStamped cavity_pose;
    cavity_pose.header = cavity_msg->header;

    if(cavity_msg->min_x > -3 && cavity_msg->max_x > -100 && cavity_msg->max_x < 3
       && cavity_msg->min_y > -3 && cavity_msg->max_y > -100 && cavity_msg->max_y < 3
       && cavity_msg->min_z > -3 && cavity_msg->max_z > -100)
    {
        ++count_time_;
        boost::chrono::high_resolution_clock::time_point t_start_all = boost::chrono::high_resolution_clock::now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

//        pcl::PointXYZ center( (cavity_msg->min_x + cavity_msg->max_x)/2, (cavity_msg->min_y + cavity_msg->max_y)/2, cavity_msg->min_z);
//        float diagonal_length = std::sqrt( (cavity_msg->min_x-cavity_msg->max_x)*(cavity_msg->min_x-cavity_msg->max_x) + (cavity_msg->min_y-cavity_msg->max_y)*(cavity_msg->min_y-cavity_msg->max_y) );
//        float radius = diagonal_length + band_width_;
//        pcl::PointXYZ c = center;

//        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//        kdtree.setInputCloud (cloud);

//        // Neighbors within radius search
//        std::vector<int> pointIdxRadiusSearch;
//        std::vector<float> pointRadiusSquaredDistance;

//        Eigen::Vector4f plane_parameters;
//        float curvature;

//        bool found_normal = false;
//        int n_trys = 0;
//        while(!found_normal && n_trys < 10)
//        {
//            if ( kdtree.radiusSearch (center, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3 )
//            {
//                //find normal from neighbor points
//                pcl::computePointNormal(*cloud, pointIdxRadiusSearch, plane_parameters, curvature);
//                pcl::flipNormalTowardsViewpoint(center,0.0,0.0,0.0, plane_parameters);

//                //find avg depth from neighbor points for better estimate of cavity entrance depth
//                pcl::CentroidPoint<pcl::PointXYZ> centroid;
//                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//                      centroid.add(cloud->points[pointIdxRadiusSearch[i]]);
//                centroid.get(c);

//                found_normal = true;
//            }
//            else
//            {
//                radius += 0.005;
//            }
//            n_trys++;
//        }

        pcl::PointXYZ center( (cavity_msg->min_x + cavity_msg->max_x)/2, (cavity_msg->min_y + cavity_msg->max_y)/2, cavity_msg->min_z);
        pcl::PointXYZ c = center;

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(octree_resolution_);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();

        // Neighbors within box search
        std::vector<int> point_idx_box_search;
        float radius = band_width_;

        Eigen::Vector4f plane_parameters;
        float curvature;

        bool found_normal = false;
        int n_trys = 0;
        while(!found_normal && n_trys < 20)
        {
            Eigen::Vector3f min_box_pt(cavity_msg->min_x - radius, cavity_msg->min_y - radius, cavity_msg->min_z);
            Eigen::Vector3f max_box_pt(cavity_msg->max_x + radius, cavity_msg->max_y + radius, cavity_msg->min_z + 2*radius);
            if ( octree.boxSearch(min_box_pt, max_box_pt, point_idx_box_search) > 3 )
            {
                //find normal from neighbor points
                pcl::computePointNormal(*cloud, point_idx_box_search, plane_parameters, curvature);
                pcl::flipNormalTowardsViewpoint(center,0.0,0.0,0.0, plane_parameters);

                //find avg depth from neighbor points for better estimate of cavity entrance depth
                pcl::CentroidPoint<pcl::PointXYZ> centroid;
                for (size_t i = 0; i < point_idx_box_search.size (); ++i)
                    centroid.add(cloud->points[point_idx_box_search[i]]);
                centroid.get(c);

                found_normal = true;
            }
            else
            {
                radius += 0.001;
            }
            n_trys++;
        }

        cavity_normal.normal.x = plane_parameters[0];
        cavity_normal.normal.y = plane_parameters[1];
        cavity_normal.normal.z = plane_parameters[2];
        cavity_normal.radius = radius;
        cavity_normal.center.x = center.x;
        cavity_normal.center.y = center.y;
        cavity_normal.center.z = c.z;

        cavity_pose.pose.position.x = center.x;
        cavity_pose.pose.position.y = center.y;
        cavity_pose.pose.position.z = c.z;
        Eigen::Vector3f normal;
        normal << plane_parameters[0], plane_parameters[1], plane_parameters[2];
        Eigen::Quaternionf orientation;
        orientation.setFromTwoVectors(Eigen::Vector3f::UnitZ(),normal);
        cavity_pose.pose.orientation.x = orientation.x();
        cavity_pose.pose.orientation.y = orientation.y();
        cavity_pose.pose.orientation.z = orientation.z();
        cavity_pose.pose.orientation.w = orientation.w();

        boost::chrono::high_resolution_clock::time_point t_end_all = boost::chrono::high_resolution_clock::now();
        time_all_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_all-t_start_all);

        if(count_time_ == 500)
        {
            boost::chrono::nanoseconds avg_time_all = time_all_ / count_time_;
            NODELET_DEBUG_STREAM("Time cavity normal : all=" << avg_time_all << " (" << (int)(1.0 / (avg_time_all.count()*1e-9)) << " Hz)");
            time_all_ = boost::chrono::nanoseconds(0);
            count_time_ = 0;
        }
    }
    else
    {
        cavity_normal.normal.x = -100;
        cavity_normal.normal.y = -100;
        cavity_normal.normal.z = -100;
        cavity_normal.radius = 0;
        cavity_normal.center.x = 0;
        cavity_normal.center.y = 0;
        cavity_normal.center.z = 0;

        cavity_pose.pose.position.x = 0;
        cavity_pose.pose.position.y = 0;
        cavity_pose.pose.position.z = 0;
        cavity_pose.pose.orientation.x = 0;
        cavity_pose.pose.orientation.y = 0;
        cavity_pose.pose.orientation.z = 0;
        cavity_pose.pose.orientation.w = 0;
    }   
    pub_.publish(cavity_normal);
    cavity_pose_pub_.publish(cavity_pose);

    NODELET_DEBUG_STREAM_THROTTLE(5,"The cavity normal runs at an avg of " << avg_frequency_ << " Hz");
}

} //namespace rci_cavity_normal

PLUGINLIB_DECLARE_CLASS(rci_cavity_normal, CavityNormalNodelet, rci_cavity_normal::CavityNormalNodelet, nodelet::Nodelet);



