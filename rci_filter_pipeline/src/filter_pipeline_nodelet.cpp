#include <filter_pipeline_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rci_filter_pipeline
{
FilterPipelineNodelet::FilterPipelineNodelet()
    : avg_frequency_(0),
      start_time_(boost::posix_time::microsec_clock::local_time()),
      count_time_(0),
      time_all_(boost::chrono::nanoseconds(0)),
      time_pass_(boost::chrono::nanoseconds(0)),
      time_vox_(boost::chrono::nanoseconds(0)),
      time_rad_(boost::chrono::nanoseconds(0))
{
}

void FilterPipelineNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    nh_.param("filter_pipeline/passthrough/used", use_passthrough_, bool(true));
    nh_.param("filter_pipeline/voxelgrid/used", use_voxelgrid_, bool(true));
    nh_.param("filter_pipeline/radial/used", use_radial_, bool(true));

    nh_.param("filter_pipeline/passthrough/min", passthrough_min_, double(0.05));
    nh_.param("filter_pipeline/passthrough/max", passthrough_max_, double(1.2));

    nh_.param("filter_pipeline/voxelgrid/leafsize", leafsize_, double(0.003));

    nh_.param("filter_pipeline/radial/min_neighbors", min_neighbors_, int(2));
    nh_.param("filter_pipeline/radial/radius_search", radius_search_, double(0.8));

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filter_pipeline/output/points", 1);
    sub_ = nh_.subscribe("filter_pipeline/input/points", 1, &FilterPipelineNodelet::cloudCb, this);

    NODELET_INFO_STREAM("\n" << "============================================\n"
                    << "Initialized Filter Pipeline\n"
                    << "============================================\n"
                    << "Passthrough Filter: use_passthrough = " << use_passthrough_
                    << ", min = " << passthrough_min_ << " [m]"
                    << ", max = " << passthrough_max_  << " [m]\n"
                    << "Voxelgrid Filter: use_voxelgrid = " << use_voxelgrid_
                    << ", leafsize = " << leafsize_ << " [m]\n"
                    << "Radial Outlier Filter: use_radial = " << use_radial_
                    << ", min_neighbors = " << min_neighbors_
                    << ", radius_search = " << radius_search_  << " [m]\n");

    get_params_service_ = nh_.advertiseService("filter_pipeline/get_params", &FilterPipelineNodelet::getParams, this);
    set_params_service_ = nh_.advertiseService("filter_pipeline/set_params", &FilterPipelineNodelet::setParams, this);
}

void FilterPipelineNodelet::updateStats() {
    boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration elapsed_time = end_time - start_time_;
    if(elapsed_time.total_milliseconds() > 0) {
        frequency_.push_back((int)1000/elapsed_time.total_milliseconds());
    }

    // Maximum size of queue is 100
    if (frequency_.size() == 100) {
        int sum = std::accumulate(frequency_.begin(),frequency_.end(),0);
        avg_frequency_ = (int) sum / 100;
        frequency_.clear();
    }
}

bool FilterPipelineNodelet::getParams(rci_comm::GetFilterPipelineParamsRequest &req, rci_comm::GetFilterPipelineParamsResponse &res)
{
    res.use_passthrough = use_passthrough_;
    res.use_voxelgrid = use_voxelgrid_;
    res.use_radial = use_radial_;

    res.passthrough_min = passthrough_min_;
    res.passthrough_max = passthrough_max_;

    res.voxelgrid_leafsize = leafsize_;

    res.radial_min_neighbors = min_neighbors_;
    res.radial_radius_search = radius_search_;

    return true;
}

bool FilterPipelineNodelet::setParams(rci_comm::SetFilterPipelineParamsRequest &req, rci_comm::SetFilterPipelineParamsResponse &res)
{
    NODELET_DEBUG_STREAM("Set filter pipeline params request: use_passthrough:" << (bool)req.use_passthrough
                        << ", use_voxelgrid:" << (bool)req.use_voxelgrid
                        << ", use_radial:" << (bool)req.use_radial
                        << ", passthrough_min:" << req.passthrough_min
                        << ", passthrough_max:" << req.passthrough_max
                        << ", voxelgrid_leafsize:" << req.voxelgrid_leafsize
                        << ", radial_min_neighbors:" << req.radial_min_neighbors
                        << ", radial_radius_search:" << req.radial_radius_search);
    use_passthrough_ = req.use_passthrough;
    use_voxelgrid_ = req.use_voxelgrid;
    use_radial_ = req.use_radial;

    if(req.passthrough_min != 1000)
        passthrough_min_ = req.passthrough_min;
    if(req.passthrough_max != 1000)
        passthrough_max_ = req.passthrough_max;
    if(req.voxelgrid_leafsize != 1000)
        leafsize_ = req.voxelgrid_leafsize;
    if(req.radial_min_neighbors != 1000)
        min_neighbors_ = req.radial_min_neighbors;
    if(req.radial_radius_search != 1000)
        radius_search_ = req.radial_radius_search;

    res.result = true;
    return true;
}

void FilterPipelineNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ++count_time_;
    boost::chrono::high_resolution_clock::time_point t_start_all = boost::chrono::high_resolution_clock::now();

    FilterPipelineNodelet::updateStats();
    start_time_ = boost::posix_time::microsec_clock::local_time();

    int n_points_init = 0;
    int n_points_nan = 0;
    int n_points_pass = 0;
    int n_points_voxel = 0;
    int n_points_rad = 0;

    n_points_init = cloud_msg->width * cloud_msg->height;

    //New PCL container
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(!use_passthrough_)
    {
        //remove NaN points from the cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_nan (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud,*temp_nan, indices);
        cloud = temp_nan;

        n_points_nan = cloud->width * cloud->height;
    }

    boost::chrono::high_resolution_clock::time_point t_start_pass = boost::chrono::high_resolution_clock::now();
    if(use_passthrough_ && cloud->size() > 0)
    {
        //Pass Through Filter: filter out points outside of sensor range
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pass (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(passthrough_min_,passthrough_max_);
        pass.filter(*temp_pass);
        cloud = temp_pass;

        n_points_pass = cloud->width * cloud->height;
    }
    boost::chrono::high_resolution_clock::time_point t_end_pass = boost::chrono::high_resolution_clock::now();
    time_pass_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_pass-t_start_pass);

    boost::chrono::high_resolution_clock::time_point t_start_vox = boost::chrono::high_resolution_clock::now();
    if(use_voxelgrid_ && cloud->size() > 0)
    {
        // Voxel Grid Filter: downsample the cloud, i.e. divide the cloud into multiple cube-shaped regions with the desired resolution (voxels),
        // then all points inside every voxel are processed so only one remains.
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_vox (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(leafsize_, leafsize_, leafsize_);
        voxel.setMinimumPointsNumberPerVoxel(1);
        voxel.filter(*temp_vox);
        cloud = temp_vox;

        n_points_voxel = cloud->width * cloud->height;
    }
    boost::chrono::high_resolution_clock::time_point t_end_vox = boost::chrono::high_resolution_clock::now();
    time_vox_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_vox-t_start_vox);

    boost::chrono::high_resolution_clock::time_point t_start_rad = boost::chrono::high_resolution_clock::now();
    if(use_radial_ && cloud->size() > 0)
    {
        // Radial Outlier Filter: every indice must have a number of neighbors within a specified radius to remain in the PointCloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_rad (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> rad;
        rad.setInputCloud(cloud);
        // Set number of neighbors to consider.
        rad.setMinNeighborsInRadius(min_neighbors_);
        rad.setRadiusSearch(radius_search_);
        rad.filter(*temp_rad);
        cloud = temp_rad;

        n_points_rad = cloud->width * cloud->height;
    }
    boost::chrono::high_resolution_clock::time_point t_end_rad = boost::chrono::high_resolution_clock::now();
    time_rad_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_rad-t_start_rad);

    NODELET_DEBUG_STREAM_THROTTLE(5,"Initial Number of points : " << n_points_init
                                  << "\nAfter nan removal : " << n_points_nan
                                  << "\nAfter passthrough filter : " << n_points_pass
                                  << "\nAfter voxelgrid filter : " << n_points_voxel
                                  << "\nAfter radial outlier filter : " << n_points_rad);
    NODELET_DEBUG_STREAM_THROTTLE(5,"The filter pipeline runs at an avg of " << avg_frequency_ << " Hz");

    sensor_msgs::PointCloud2Ptr output (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud, *output);
    output->header = cloud_msg->header;
    pub_.publish(output);


    boost::chrono::high_resolution_clock::time_point t_end_all = boost::chrono::high_resolution_clock::now();
    time_all_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_all-t_start_all);

    if(count_time_ == 500)
    {
        boost::chrono::nanoseconds avg_time_all = time_all_ / 500;
        boost::chrono::nanoseconds avg_time_pass = time_pass_ / 500;
        boost::chrono::nanoseconds avg_time_vox = time_vox_ / 500;
        boost::chrono::nanoseconds avg_time_rad = time_rad_ / 500;
        NODELET_DEBUG_STREAM("Time filter pipeline : all=" << avg_time_all << " (" << (int)(1.0 / (avg_time_all.count()*1e-9)) << " Hz), pass="
                            << avg_time_pass << " (" << (int)(1.0 / (avg_time_pass.count()*1e-9)) << " Hz), vox="
                            << avg_time_vox << " (" << (int)(1.0 / (avg_time_vox.count()*1e-9)) << " Hz), rad="
                            << avg_time_rad << " (" << (int)(1.0 / (avg_time_rad.count()*1e-9)) << " Hz)");
        time_all_ = time_pass_ = time_vox_ = time_rad_ = boost::chrono::nanoseconds(0);
        count_time_ = 0;
    }
}

} //namespace rci_filter_pipeline

PLUGINLIB_DECLARE_CLASS(rci_filter_pipeline, FilterPipelineNodelet, rci_filter_pipeline::FilterPipelineNodelet, nodelet::Nodelet);

