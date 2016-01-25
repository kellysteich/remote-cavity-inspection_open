#include <box_check_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rci_pc_cavity_check
{

BoxCheckNodelet::BoxCheckNodelet()
    : avg_frequency_(0),
      start_time_(boost::posix_time::microsec_clock::local_time()),
      count_time_(0),
      count_start_(0),
      count_x_(0),
      count_y_(0),
      count_z_(0),
      time_all_(boost::chrono::nanoseconds(0)),
      time_start_(boost::chrono::nanoseconds(0)),
      time_x_(boost::chrono::nanoseconds(0)),
      time_y_(boost::chrono::nanoseconds(0)),
      time_z_(boost::chrono::nanoseconds(0))
{
}

void BoxCheckNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    nh_.param("box_check/octree/resolution", octree_resolution_, double(0.001));
    nh_.param("box_check/start_box/width", start_box_width_, double(0.005));
    nh_.param("box_check/start_box/height", start_box_height_, double(0.005));
    nh_.param("box_check/start_box/length", start_box_length_, double(0.01));
    nh_.param("box_check/initial_step", initial_step_, double(0.03));
    nh_.param("box_check/max_n_inliers", max_n_inliers_, int(2));

    NODELET_INFO_STREAM("\n" << "=============================================\n"
                    << "Initialized Box Check\n"
                    << "=============================================\n"
                    << "octree/resolution = " << octree_resolution_ << " [m]\n"
                    << "start_box/width = " << start_box_width_ << " [m]"
                    << ", start_box/height = " << start_box_height_  << " [m]"
                    << ", start_box/length = " << start_box_length_ << " [m]\n"
                    << "initial_step = " << initial_step_ << " [m]"
                    << ", max_n_inliers = " << max_n_inliers_ << "\n");

    pub_ = nh_.advertise<rci_comm::CavityBox>("box_check/output/cavity_box", 1);

    cloud_sub_.subscribe(nh_, "box_check/input/points", 4);
    cavity_sub_.subscribe(nh_, "box_check/input/cavity", 4);
    sync_ = boost::make_shared<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PointStamped> > (8);
    sync_->connectInput (cloud_sub_, cavity_sub_);
    sync_->registerCallback (bind (&BoxCheckNodelet::cb, this, _1, _2));

    get_params_service_ = nh_.advertiseService("box_check/get_params", &BoxCheckNodelet::getParams, this);
    set_params_service_ = nh_.advertiseService("box_check/set_params", &BoxCheckNodelet::setParams, this);
}

void BoxCheckNodelet::updateStats()
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

bool BoxCheckNodelet::getParams(rci_comm::GetBoxCheckParamsRequest &req, rci_comm::GetBoxCheckParamsResponse &res)
{
    res.octree_resolution = octree_resolution_;
    res.initial_step = initial_step_;
    res.max_n_inliers = max_n_inliers_;
    res.start_box_width = start_box_width_;
    res.start_box_height = start_box_height_;
    res.start_box_length = start_box_length_;

    return true;
}

bool BoxCheckNodelet::setParams(rci_comm::SetBoxCheckParamsRequest &req, rci_comm::SetBoxCheckParamsResponse &res)
{
    NODELET_DEBUG_STREAM("Set box check params request: octree_resolution:" << req.octree_resolution
                        << ", initial_step:" << req.initial_step
                        << ", max_n_inliers:" << req.max_n_inliers
                        << ", start_box_width:" << req.start_box_width
                        << ", start_box_height:" << req.start_box_height
                        << ", start_box_length:" << req.start_box_length);
    if(req.octree_resolution != 1000)
        octree_resolution_ = req.octree_resolution;
    if(req.initial_step != 1000)
        initial_step_ = req.initial_step;
    if(req.max_n_inliers != 1000)
        max_n_inliers_ = req.max_n_inliers;
    if(req.start_box_width != 1000)
        start_box_width_ = req.start_box_width;
    if(req.start_box_height != 1000)
        start_box_height_ = req.start_box_height;
    if(req.start_box_length != 1000)
        start_box_length_ = req.start_box_length;

    res.result = true;
    return true;
}

void BoxCheckNodelet::cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const geometry_msgs::PointStampedConstPtr& cavity_msg)
{
    BoxCheckNodelet::updateStats();
    start_time_ = boost::posix_time::microsec_clock::local_time();

    rci_comm::CavityBox box;
    if(cavity_msg->point.x != 0 && cavity_msg->point.y != 0 && cavity_msg->point.z != 0)
    {
        ++count_time_;
        boost::chrono::high_resolution_clock::time_point t_start_all = boost::chrono::high_resolution_clock::now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        pcl::PointXYZ center(cavity_msg->point.x, cavity_msg->point.y, cavity_msg->point.z);

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(octree_resolution_);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();

        // Neighbors within box search
        std::vector<int> point_idx;
        Eigen::Vector3f min_box_pt(center.x-(start_box_width_/2), center.y-(start_box_height_/2), center.z-(start_box_length_/2));
        Eigen::Vector3f max_box_pt(center.x+(start_box_width_/2), center.y+(start_box_height_/2), center.z+(start_box_length_/2));

        if(octree.boxSearch(min_box_pt, max_box_pt, point_idx) > 0)
        {
            NODELET_WARN_STREAM("first box didn't fit! " << point_idx.size());
            box.header = cavity_msg->header;
            box.min_x = -100;
            box.min_y = -100;
            box.min_z = -100;
            box.max_x = -100;
            box.max_y = -100;
            box.max_z = -100;
            pub_.publish(box);
        }
        else
        {
            ++count_start_;
            boost::chrono::high_resolution_clock::time_point t_start_start = boost::chrono::high_resolution_clock::now();
            //std::cout << "first box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ")" << std::endl;
            bool fits = true;
            double step = initial_step_;
            Eigen::Vector3f new_min_box_pt = min_box_pt;
            Eigen::Vector3f new_max_box_pt = max_box_pt;
            while (fits && new_min_box_pt[0] > -3 && step >= 0.001)
            {
                new_min_box_pt[0] -= step;
                new_min_box_pt[1] -= step;
                new_max_box_pt[0] += step;
                new_max_box_pt[1] += step;
                new_max_box_pt[2] += step;
                //std::cout << "step: " << step << std::endl;
                if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) > max_n_inliers_)
                {
                    //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                    if(step <= 0.001)
                    {
                        fits = false;
                        //std::cout << "step too small, stop." << std::endl;
                    }
                    else
                    {
                        bool fits_not = true;
                        step = step / 2;
                        while(fits_not && new_min_box_pt[0] < min_box_pt[0])
                        {
                            new_min_box_pt[0] += step;
                            new_min_box_pt[1] += step;
                            new_max_box_pt[0] -= step;
                            new_max_box_pt[1] -= step;
                            new_max_box_pt[2] -= step;
                            //std::cout << "minus step: " << step << std::endl;
                            if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) <= max_n_inliers_)
                            {
                                min_box_pt = new_min_box_pt;
                                max_box_pt = new_max_box_pt;
                                fits_not = false;
                                //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                            else
                            {
                                //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                        }
                        step = step / 2;
                        //std::cout << "reduce step: " << step << std::endl;
                    }
                }
                else
                {
                    min_box_pt = new_min_box_pt;
                    max_box_pt = new_max_box_pt;
                    //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                }
            }
            //std::cout << "box is now: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ")" << std::endl;
            boost::chrono::high_resolution_clock::time_point t_end_start = boost::chrono::high_resolution_clock::now();
            time_start_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_start-t_start_start);

            ++count_x_;
            boost::chrono::high_resolution_clock::time_point t_start_x = boost::chrono::high_resolution_clock::now();
            //try extending in pos x direction
            fits = true;
            new_min_box_pt = min_box_pt;
            new_max_box_pt = max_box_pt;
            step = initial_step_/4;
            //std::cout << "extending in pos x" << std::endl;
            while (fits && new_max_box_pt[0] < 3  && step >= 0.001)
            {
                new_max_box_pt[0] += step;
                //std::cout << "step: " << step << std::endl;
                if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) > max_n_inliers_)
                {
                    //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                    if(step <= 0.001)
                    {
                        fits = false;
                        //std::cout << "step too small, stop." << std::endl;
                    }
                    else
                    {
                        bool fits_not = true;
                        step = step / 2;
                        while(fits_not && new_max_box_pt[0] > max_box_pt[0])
                        {
                            new_max_box_pt[0] -= step;
                            //std::cout << "minus step: " << step << std::endl;
                            if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) <= max_n_inliers_)
                            {
                                min_box_pt = new_min_box_pt;
                                max_box_pt = new_max_box_pt;
                                fits_not = false;
                                //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                            else
                            {
                                //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                        }
                        step = step / 2;
                        //std::cout << "reduce step: " << step << std::endl;
                    }
                }
                else
                {
                    min_box_pt = new_min_box_pt;
                    max_box_pt = new_max_box_pt;
                    //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                }
            }
            //std::cout << "box is now: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ")" << std::endl;

            //try extending in neg x direction
            fits = true;
            new_min_box_pt = min_box_pt;
            new_max_box_pt = max_box_pt;
            step = initial_step_/4;
            //std::cout << "extending in neg x" << std::endl;
            while (fits && new_min_box_pt[0] > -3  && step >= 0.001)
            {
                new_min_box_pt[0] -= step;
                //std::cout << "step: " << step << std::endl;
                if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) > max_n_inliers_)
                {
                    //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                    if(step <= 0.001)
                    {
                        fits = false;
                        //std::cout << "step too small, stop." << std::endl;
                    }
                    else
                    {
                        bool fits_not = true;
                        step = step / 2;
                        while(fits_not && new_min_box_pt[0] < min_box_pt[0])
                        {
                            new_min_box_pt[0] += step;
                            //std::cout << "minus step: " << step << std::endl;
                            if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) <= max_n_inliers_)
                            {
                                min_box_pt = new_min_box_pt;
                                max_box_pt = new_max_box_pt;
                                fits_not = false;
                                //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                            else
                            {
                                //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                        }
                        step = step / 2;
                        //std::cout << "reduce step: " << step << std::endl;
                    }
                }
                else
                {
                    min_box_pt = new_min_box_pt;
                    max_box_pt = new_max_box_pt;
                    //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                }
            }
            //std::cout << "box is now: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ")" << std::endl;
            boost::chrono::high_resolution_clock::time_point t_end_x = boost::chrono::high_resolution_clock::now();
            time_x_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_x-t_start_x);

            ++count_y_;
            boost::chrono::high_resolution_clock::time_point t_start_y = boost::chrono::high_resolution_clock::now();
            //try extending in pos y direction
            fits = true;
            new_min_box_pt = min_box_pt;
            new_max_box_pt = max_box_pt;
            step = initial_step_/4;
            //std::cout << "extending in pos y" << std::endl;
            while (fits && new_max_box_pt[1] < 3  && step >= 0.001)
            {
                new_max_box_pt[1] += step;
                //std::cout << "step: " << step << std::endl;
                if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) > max_n_inliers_)
                {
                    //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                    if(step <= 0.001)
                    {
                        fits = false;
                        //std::cout << "step too small, stop." << std::endl;
                    }
                    else
                    {
                        bool fits_not = true;
                        step = step / 2;
                        while(fits_not && new_max_box_pt[1] > max_box_pt[1])
                        {
                            new_max_box_pt[1] -= step;
                            //std::cout << "minus step: " << step << std::endl;
                            if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) <= max_n_inliers_)
                            {
                                min_box_pt = new_min_box_pt;
                                max_box_pt = new_max_box_pt;
                                fits_not = false;
                                //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                            else
                            {
                                //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                        }
                        step = step / 2;
                        //std::cout << "reduce step: " << step << std::endl;
                    }
                }
                else
                {
                    min_box_pt = new_min_box_pt;
                    max_box_pt = new_max_box_pt;
                    //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                }
            }
            //std::cout << "box is now: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ")" << std::endl;

            //try extending in neg y direction
            fits = true;
            new_min_box_pt = min_box_pt;
            new_max_box_pt = max_box_pt;
            step = initial_step_/4;
            //std::cout << "extending in neg y" << std::endl;
            while (fits && new_min_box_pt[1] > -3  && step >= 0.001)
            {
                new_min_box_pt[1] -= step;
                //std::cout << "step: " << step << std::endl;
                if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) > max_n_inliers_)
                {
                    //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                    if(step <= 0.001)
                    {
                        fits = false;
                        //std::cout << "step too small, stop." << std::endl;
                    }
                    else
                    {
                        bool fits_not = true;
                        step = step / 2;
                        while(fits_not && new_min_box_pt[1] < min_box_pt[1])
                        {
                            new_min_box_pt[1] += step;
                            //std::cout << "minus step: " << step << std::endl;
                            if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) <= max_n_inliers_)
                            {
                                min_box_pt = new_min_box_pt;
                                max_box_pt = new_max_box_pt;
                                fits_not = false;
                                //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                            else
                            {
                                //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                        }
                        step = step / 2;
                        //std::cout << "reduce step: " << step << std::endl;
                    }
                }
                else
                {
                    min_box_pt = new_min_box_pt;
                    max_box_pt = new_max_box_pt;
                    //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                }
            }
            //std::cout << "box is now: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ")" << std::endl;
            boost::chrono::high_resolution_clock::time_point t_end_y = boost::chrono::high_resolution_clock::now();
            time_y_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_y-t_start_y);

            ++count_z_;
            boost::chrono::high_resolution_clock::time_point t_start_z = boost::chrono::high_resolution_clock::now();
            //try extending in pos z direction
            fits = true;
            new_min_box_pt = min_box_pt;
            new_max_box_pt = max_box_pt;
            step = initial_step_;
            //std::cout << "extending in pos z" << std::endl;
            while (fits && new_max_box_pt[2] < 3 && step >= 0.001)
            {
                new_max_box_pt[2] += step;
                //std::cout << "step: " << step << std::endl;
                if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) > (max_n_inliers_*2))
                {
                    //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                    if(step <= 0.001)
                    {
                        fits = false;
                        //std::cout << "step too small, stop." << std::endl;
                    }
                    else
                    {
                        bool fits_not = true;
                        step = step / 2;
                        while(fits_not && new_max_box_pt[2] > max_box_pt[2])
                        {
                            new_max_box_pt[2] -= step;
                            //std::cout << "minus step: " << step << std::endl;
                            if(octree.boxSearch(new_min_box_pt, new_max_box_pt, point_idx) <= (max_n_inliers_*2))
                            {
                                min_box_pt = new_min_box_pt;
                                max_box_pt = new_max_box_pt;
                                fits_not = false;
                                //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                            else
                            {
                                //std::cout << "box did not fit: (" << new_min_box_pt[0] << "," << new_max_box_pt[0] << ") (" << new_min_box_pt[1] << "," << new_max_box_pt[1] << ") (" << new_min_box_pt[2] << "," << new_max_box_pt[2] << ") " << point_idx.size() << std::endl;
                            }
                        }
                        step = step / 2;
                        //std::cout << "reduce step: " << step << std::endl;
                    }
                }
                else
                {
                    min_box_pt = new_min_box_pt;
                    max_box_pt = new_max_box_pt;
                    //std::cout << "box fit: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ") " << point_idx.size() << std::endl;
                }
            }
            //std::cout << "box is now: (" << min_box_pt[0] << "," << max_box_pt[0] << ") (" << min_box_pt[1] << "," << max_box_pt[1] << ") (" << min_box_pt[2] << "," << max_box_pt[2] << ")" << std::endl;
            boost::chrono::high_resolution_clock::time_point t_end_z = boost::chrono::high_resolution_clock::now();
            time_z_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_z-t_start_z);

            box.header = cavity_msg->header;
            box.min_x = min_box_pt[0];
            box.min_y = min_box_pt[1];
            box.min_z = center.z; //min_box_pt[2];
            box.max_x = max_box_pt[0];
            box.max_y = max_box_pt[1];
            box.max_z = max_box_pt[2];
            pub_.publish(box);
        }
        boost::chrono::high_resolution_clock::time_point t_end_all = boost::chrono::high_resolution_clock::now();
        time_all_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_all-t_start_all);
    }
    else
    {
        //std::cout << "no cavities" << std::endl;
        box.header = cavity_msg->header;
        box.min_x = -100;
        box.min_y = -100;
        box.min_z = -100;
        box.max_x = -100;
        box.max_y = -100;
        box.max_z = -100;
        pub_.publish(box);
    }   

    NODELET_DEBUG_STREAM_THROTTLE(5,"The box check runs at an avg of " << avg_frequency_ << " Hz");

    if(count_time_ == 500)
    {
        boost::chrono::nanoseconds avg_time_all = time_all_ / count_time_;
        boost::chrono::nanoseconds avg_time_start = time_start_ / count_start_;
        boost::chrono::nanoseconds avg_time_x = time_x_ / count_x_;
        boost::chrono::nanoseconds avg_time_y = time_y_ / count_y_;
        boost::chrono::nanoseconds avg_time_z = time_z_ / count_z_;
        NODELET_DEBUG_STREAM("Time box check : all=" << avg_time_all << " (" << (int)(1.0 / (avg_time_all.count()*1e-9)) << " Hz), start="
                            << avg_time_start << " (" << (int)(1.0 / (avg_time_start.count()*1e-9)) << " Hz), x="
                            << avg_time_x << " (" << (int)(1.0 / (avg_time_x.count()*1e-9)) << " Hz), y="
                            << avg_time_y << " (" << (int)(1.0 / (avg_time_y.count()*1e-9)) << " Hz), z="
                            << avg_time_z << " (" << (int)(1.0 / (avg_time_z.count()*1e-9)) << " Hz)");
        time_all_ = time_start_ = time_x_ = time_y_ = time_z_ = boost::chrono::nanoseconds(0);
        count_time_ = 0;
    }
}

} //namespace rci_pc_cavity_check

PLUGINLIB_DECLARE_CLASS(rci_pc_cavity_check, BoxCheckNodelet, rci_pc_cavity_check::BoxCheckNodelet, nodelet::Nodelet);



