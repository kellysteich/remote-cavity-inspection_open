#include <tree_cavity_detection_kmeans_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rci_img_tree_detection
{

TreeCavityDetectionKmeansNodelet::TreeCavityDetectionKmeansNodelet()
    : prev_seedpoint_depth_(0.0),
      n_skipped_(0),
      first_run_(true),
      avg_frequency_(0),
      start_time_(boost::posix_time::microsec_clock::local_time()),
      rng_(cv::theRNG()),
      count_time_(0),
      time_all_(boost::chrono::nanoseconds(0))
{
}

void TreeCavityDetectionKmeansNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    nh_.param("is_simulated", is_simulated_, bool(false));

    nh_.param("sensor/depth/max", sensor_depth_max_, int(5));
    nh_.param("sensor/depth/scale", depth_scale_, double(0.0002));

    nh_.param("img_tree_detection/debug", debug_, bool(false));

    nh_.param("img_tree_detection/min_depth", min_depth_, double(0.25));
    nh_.param("img_tree_detection/max_depth", max_depth_, double(2.5));

    nh_.param("img_tree_detection/tree/thresh_seed", thresh_seed_, double(0.1));
    nh_.param("img_tree_detection/tree/thresh_depth", thresh_depth_, double(0.1));
    nh_.param("img_tree_detection/tree/max_scaling", max_scaling_, double(1.7));

    nh_.param("img_tree_detection/cavity/min_width", min_cavity_width_, double(0.03));
    nh_.param("img_tree_detection/cavity/min_height", min_cavity_height_, double(0.03));

    nh_.param("img_tree_detection/kmeans/k", k_, int(10));
    nh_.param("img_tree_detection/kmeans/tol", kmeans_tol_, double(0.001));
    nh_.param("img_tree_detection/kmeans/max_trys", kmeans_max_trys_, int(50));

    if(is_simulated_)
        depth_scale_ = 0.001;

    ROS_INFO_STREAM("Wait for max. 3 second to get picoflexx Camera Info.");
    sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/img_tree_detection/input/cam_info", nh_, ros::Duration(3));
    if (cam_info == NULL)
    {
        depth_fxinv_ = 1.0f/216.47f;
        depth_fyinv_ = 1.0f/216.47f;
        depth_cx_ = 107.727f;
        depth_cy_ = 86.973f;
        ROS_WARN_STREAM("\n" << "====================================\n"
                        << "        Camera information          \n"
                        << "====================================\n"
                        << "Failed to get Camera Info from topic, initializing with default parameters!\n"
                        << "depth_fxinv = " << depth_fxinv_
                        << ", depth_fyinv = " << depth_fyinv_ << "\n"
                        << "depth_cx = " << depth_cx_
                        << ", depth_cy = " << depth_cy_ << "\n");
    }
    else
    {
        depth_fxinv_ = 1.0f/cam_info->P[0];
        depth_fyinv_ = 1.0f/cam_info->P[5];
        depth_cx_ = cam_info->P[2];
        depth_cy_ = cam_info->P[6];
        ROS_INFO_STREAM("\n" << "====================================\n"
                        << "        Camera information          \n"
                        << "====================================\n"
                        << "Successfully got Camera Info from topic!\n"
                        << "depth_fxinv = " << depth_fxinv_
                        << ", depth_fyinv = " << depth_fyinv_ << "\n"
                        << "depth_cx = " << depth_cx_
                        << ", depth_cy = " << depth_cy_ << "\n");
    }

    thresh_depth_scaled_ = thresh_depth_ / depth_scale_;
    thresh_seed_scaled_ = thresh_seed_ / depth_scale_;
    sensor_depth_max_scaled_ = sensor_depth_max_ / depth_scale_;
    kmeans_tol_scaled_ = kmeans_tol_ / depth_scale_;
    if(debug_)
        std::cout << "thresh_seed scaled=" << thresh_seed_scaled_
                  << ", thresh_depth scaled=" << thresh_depth_scaled_
                  << ", sensor_depth_max scaled=" << sensor_depth_max_scaled_
                  << ", kmeans_tol scaled=" << kmeans_tol_scaled_ << std::endl;

    seedpoint_.x = 0;
    seedpoint_.y = 0;

    NODELET_INFO_STREAM("\n" << "============================================\n"
                    << "Initialized Tree Cavity Detection Kmeans\n"
                    << "============================================\n"
                    << "sensor/name = picoflexx"
                    << ", is_simulated = " << is_simulated_ << "\n"
                    << "sensor/depth_scale = " << depth_scale_
                    << ", sensor/depth_max = " << sensor_depth_max_ << " [m]\n"
                    << "min_depth = " << min_depth_ << " [m]"
                    << ", max_depth = " << max_depth_<< " [m]\n"
                    << "tree/thresh_seed = " << thresh_seed_ << " [m]"
                    << ", tree/thresh_depth = " << thresh_depth_  << " [m]"
                    << ", tree/max_scaling = " << max_scaling_ << "\n"
                    << "cavity/min_width = " << min_cavity_width_ << " [m]"
                    << ", cavity/min_height = " << min_cavity_height_ << " [m]\n"
                    << "kmeans/k = " << k_
                    << ", kmeans/tol = " << kmeans_tol_ << " [m]"
                    << ", kmeans/max_trys = " << kmeans_max_trys_ << "\n");

    tree_pub_ = nh_.advertise<rci_comm::Tree>("img_tree_detection/output/tree", 1);
    cavity_pub_ = nh_.advertise<geometry_msgs::PointStamped>("img_tree_detection/output/cavity", 1);
    lost_pub_ = nh_.advertise<std_msgs::Bool>("img_tree_detection/lost_tree", 1);

    image_transport::ImageTransport it(nh_);
    image_pub_ = it.advertise("img_tree_detection/output/image", 1);
    image_sub_ = it.subscribe("img_tree_detection/input/image", 1, &TreeCavityDetectionKmeansNodelet::imageCb, this);

    seed_service_ = nh_.advertiseService("img_tree_detection/init_seed_point", &TreeCavityDetectionKmeansNodelet::initSeedPoint, this);
    get_params_service_ = nh_.advertiseService("img_tree_detection/get_params", &TreeCavityDetectionKmeansNodelet::getParams, this);
    set_params_service_ = nh_.advertiseService("img_tree_detection/set_params", &TreeCavityDetectionKmeansNodelet::setParams, this);
}


bool TreeCavityDetectionKmeansNodelet::initSeedPoint(rci_comm::InitSeedPoint::Request &req, rci_comm::InitSeedPoint::Response &res)
{
    seedpoint_ = req.seedpoint;
    first_run_ = true;
    n_skipped_ = 0;

    res.result = true;

    NODELET_DEBUG_STREAM("Init seed point request:\n" << req.seedpoint);
    return true;
}

bool TreeCavityDetectionKmeansNodelet::getParams(rci_comm::GetTreeDetectionParamsRequest &req, rci_comm::GetTreeDetectionParamsResponse &res)
{
    res.min_depth = min_depth_;
    res.thresh_seed = thresh_seed_;
    res.thresh_depth = thresh_depth_;
    res.max_scaling = max_scaling_;

    res.min_cavity_width = min_cavity_width_;
    res.min_cavity_height = min_cavity_height_;

    res.k = k_;
    res.tol = kmeans_tol_;
    res.max_trys = kmeans_max_trys_;

    return true;
}

bool TreeCavityDetectionKmeansNodelet::setParams(rci_comm::SetTreeDetectionParamsRequest &req, rci_comm::SetTreeDetectionParamsResponse &res)
{
    NODELET_DEBUG_STREAM("Set tree detection params request: min_depth:" << req.min_depth
                        << ", tresh_seed:" << req.thresh_seed
                        << ", thresh_depth:" << req.thresh_depth
                        << ", max_scaling:" << req.thresh_depth
                        << ", min_cavity_width:" << req.thresh_depth
                        << ", min_cavity_height:" << req.thresh_depth
                        << ", k:" << req.thresh_depth
                        << ", tol:" << req.thresh_depth
                        << ", max_trys:" << req.thresh_depth);
    if(req.min_depth != 1000)
    {
        min_depth_ = req.min_depth;
    }
    if(req.thresh_seed != 1000)
    {
        thresh_seed_ = req.thresh_seed;
        thresh_seed_scaled_ = thresh_depth_ / depth_scale_;
    }
    if(req.thresh_depth != 1000)
    {
        thresh_depth_ = req.thresh_depth;
        thresh_depth_scaled_ = thresh_depth_ / depth_scale_;
    }
    if(req.max_scaling != 1000)
        max_scaling_ = req.max_scaling;
    if(req.min_cavity_width != 1000)
        min_cavity_width_ = req.min_cavity_width;
    if(req.min_cavity_height != 1000)
        min_cavity_height_ = req.min_cavity_height;
    if(req.k != 1000)
        k_ = req.k;
    if(req.tol != 1000)
    {
        kmeans_tol_ = req.tol;
        kmeans_tol_scaled_ = kmeans_tol_ / depth_scale_;
    }
    if(req.max_trys != 1000)
        kmeans_max_trys_ = req.max_trys;

    res.result = true;
    return true;
}

bool TreeCavityDetectionKmeansNodelet::findNewSeedPoint(const cv_bridge::CvImageConstPtr &img)
{
    bool result = true;
    //Get moments of previous tree contour
    cv::Moments mu = cv::moments(prev_tree_contour_, false );
    //mass center:
    cv::Point2f mc = cv::Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
    seedpoint_.x = mc.x;
    seedpoint_.y = mc.y-10; //slightly up to avoid robot arm

    //check if value of new seedpoint isn't too different from the old seedpoint's value
    int diff = abs(prev_seedpoint_depth_ - (double) img->image.at<unsigned short>(seedpoint_.y,seedpoint_.x));
    //if it is too different, try moving the seedpoint around to find a point with a similar value
    if(diff > thresh_seed_scaled_)
    {
        int n_trys = 1; //number of times the seedpoint has been moved in all 4 directions
        int dir = 1;
        geometry_msgs::Point new_seed = seedpoint_;
        //move the seedpoint until color is similar enough to the previous seedpoint color or until we reach the max number of trys (25)
        while(diff > thresh_seed_scaled_ && n_trys < 100)
        {
            //shift seed point
            switch(dir)
            {
            case 1 :
                if(seedpoint_.x + n_trys < img->image.cols)
                {
                    new_seed.x = seedpoint_.x + n_trys;
                    new_seed.y = seedpoint_.y;
                }
                dir = 2;
                break;
            case 2 :
                if(seedpoint_.y + n_trys < img->image.rows)
                {
                    new_seed.x = seedpoint_.x;
                    new_seed.y = seedpoint_.y + n_trys;
                }
                dir = 3;
                break;
            case 3 :
                if(seedpoint_.x - n_trys > 0)
                {
                    new_seed.x = seedpoint_.x - n_trys;
                    new_seed.y = seedpoint_.y;
                }
                dir = 4;
                break;
            case 4 :
                if(seedpoint_.y - n_trys > 0)
                {
                    new_seed.x = seedpoint_.x;
                    new_seed.y = seedpoint_.y - n_trys;
                }
                dir = 1;
                n_trys++;
                break;
            default :
                break;
            }
            diff = abs(prev_seedpoint_depth_ - (double) img->image.at<unsigned short>(new_seed.y,new_seed.x));
        }
        //if we didn't find a good seedpoint, return false and increase n_skipped
        if(diff > thresh_seed_scaled_)
        {
            result = false;
            n_skipped_++;
            ROS_INFO_STREAM("SKIPPING! No seedpoint.");
        }
        //if we did find a good seedpoint, return true and update the seedpoint
        else
        {
            result = true;
            ROS_DEBUG_STREAM("Changing seed point from [" << seedpoint_.x << "," << seedpoint_.y << "] to [" << new_seed.x << "," << new_seed.y << "]");
            seedpoint_ = new_seed;
        }
    }
    return result;
}

bool TreeCavityDetectionKmeansNodelet::findTree(const cv_bridge::CvImageConstPtr &img, int &id, std::vector<cv::Point> &contour,
                                           std::vector<std::vector<cv::Point> > &contours, std::vector<cv::Vec4i> &hierarchy)
{
    cv::Mat tempframe;
    img->image.convertTo(tempframe, CV_8UC1);

    //threshold image
    for(int j = 0; j < img->image.rows; j++){
        for(int i = 0; i < img->image.cols; i++){
            double depth = (double)img->image.at<unsigned short>(j,i)*depth_scale_;
            if(depth < min_depth_ || depth > max_depth_)
            {
                tempframe.at<uchar>(j,i) = (uchar)0;
            }
        }
    }

    std::vector<cv::Point> nonzero_locations;
    cv::findNonZero(tempframe, nonzero_locations);

    std::vector<double> cluster_centers(k_,0.0);
    generateRandomNotSeedCenter(cluster_centers, nonzero_locations, img);

    std::vector<std::vector<cv::Point>> clusters(k_);
    kmeans(clusters, cluster_centers, nonzero_locations, img);

    //Find the cluster with the center closest to the seedpoint
    int min_cluster_id;
    findClosestSeedpointTreeCluster(cluster_centers, min_cluster_id, img);

    //Merge clusters that are close enough to the min_cluster
    //then create binary image based on whether a point is in the merged cluster or not
    cv::Mat thresholded = findTreeBinary(clusters, cluster_centers, min_cluster_id, img);

    if(debug_)
        cv::imshow("Thresholded", thresholded);

    cv::Mat thresholded_erode_dilate = cv::Mat::zeros(thresholded.size(), CV_8UC1);
    cv::dilate(thresholded, thresholded_erode_dilate, cv::Mat(), cv::Point(-1,-1), 2);
    cv::erode(thresholded_erode_dilate, thresholded_erode_dilate, cv::Mat(), cv::Point(-1,-1), 2);

    if(debug_)
        cv::imshow("Thresholded erode dilate", thresholded_erode_dilate);

    // Find contours
    cv::findContours(thresholded_erode_dilate.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    if(debug_)
    {
        // Draw contour
        cv::Mat debug_contours = cv::Mat::zeros(thresholded.size(), CV_8UC3);
        for(int i = 0; i < contours.size(); i++ )
        {
            cv::Scalar color = cv::Scalar( rng_.uniform(0, 255), rng_.uniform(0,255), rng_.uniform(0,255) );
            cv::drawContours(debug_contours, contours, i, color, 2);
        }
        cv::imshow("All Contours", debug_contours);
    }

    if(debug_)
        cv::waitKey(3);

    //Find contour which contains the seedpoint (and has no parent contour)
    findSeedpointContour(contour, id, contours, hierarchy, seedpoint_);
    if(contour.size() > 2)
        return true;
    else
        return false;
}

void TreeCavityDetectionKmeansNodelet::generateRandomNotSeedCenter(std::vector<double> &center, const std::vector<cv::Point> &samples, const cv_bridge::CvImageConstPtr &img)
{
    double seedpoint_value = (double)img->image.at<unsigned short>(seedpoint_.y,seedpoint_.x);
    center[0] = seedpoint_value;
    for(int c = 1; c < center.size(); c++) {
        bool not_good = true;
        double rnd_center;
        while(not_good) {
            int index = rng_.uniform(0,samples.size());
            rnd_center  = (double)img->image.at<unsigned short>(samples[index]);
            not_good = false;
            for(int i = 0; i < c; i++) {
                if(rnd_center == center[i]) {
                    not_good = true;
                }
            }
        }
        center[c] = rnd_center;
    }
}

void TreeCavityDetectionKmeansNodelet::kmeans(std::vector<std::vector<cv::Point>> &clusters, std::vector<double> &cluster_centers, const std::vector<cv::Point> &nonzero_locations, const cv_bridge::CvImageConstPtr &img)
{
    bool converged = false;
    int nTrys = 0;
    //do kmeans clustering
    while(!converged && nTrys < kmeans_max_trys_) //while not converged
    {
        //assign each point to its closest center
        for(int i = 0; i < nonzero_locations.size(); i++)
        {
            double point_value = (double)img->image.at<unsigned short>(nonzero_locations[i]);
            double min_dist = std::abs(point_value - cluster_centers[0]);
            int cluster_id = 0;
            //calculate distance to each center to find closest center
            for(int c = 1; c < cluster_centers.size(); c++)
            {
                double dist = std::abs(point_value - cluster_centers[c]);
                if(dist < min_dist)
                {
                    min_dist = dist;
                    cluster_id = c;
                }
            }
            clusters[cluster_id].push_back(nonzero_locations[i]);
        }
        //move each center to the average of its assigned points
        std::vector<double> new_cluster_centers(cluster_centers.size());
        std::vector<double> cluster_moved_distance(cluster_centers.size());
        for(int c = 0; c < cluster_centers.size(); c++)
        {
            if(clusters[c].size() > 0)
            {
                double sum = 0;
                for(int i = 0; i < clusters[c].size(); i++)
                {
                    sum += (double)img->image.at<unsigned short>(clusters[c][i]);
                }
                new_cluster_centers[c] = sum / clusters[c].size();
            }
            else
            {
                new_cluster_centers[c] = cluster_centers[c];
            }
            cluster_moved_distance[c] = std::abs(cluster_centers[c] - new_cluster_centers[c]);
        }
        //check for convergence (cluster don't move more than a tolerance anymore)
        if(*std::max_element(cluster_moved_distance.begin(),cluster_moved_distance.end()) < kmeans_tol_scaled_)
        {
            converged = true;
        }
        cluster_centers = new_cluster_centers;
        nTrys++;
    }
}

void TreeCavityDetectionKmeansNodelet::findClosestSeedpointTreeCluster(std::vector<double> &cluster_centers, int &tree_id, const cv_bridge::CvImageConstPtr &img)
{
    double seedpoint_value = (double)img->image.at<unsigned short>(seedpoint_.y,seedpoint_.x);
    tree_id = 0;
    double min_dist = std::abs(cluster_centers[0] - seedpoint_value);
    //calculate distance to each center to find closest center
    for(int c = 1; c < cluster_centers.size(); c++)
    {
        if(std::abs(cluster_centers[c]-seedpoint_value) < min_dist)
        {
            tree_id = c;
            min_dist = std::abs(cluster_centers[c]-seedpoint_value);
        }
    }
}

cv::Mat TreeCavityDetectionKmeansNodelet::findTreeBinary(std::vector<std::vector<cv::Point>> &clusters, std::vector<double> &cluster_centers, int &tree_id, const cv_bridge::CvImageConstPtr &img)
{
    cv::Mat debug_clusters;
    if(debug_)
    {
        //Draw clusters (cluster closest to seedpoint (tree cluster) is pink, merged clusters will be shades of blue)
        debug_clusters = cv::Mat::zeros(img->image.size(), CV_8UC3);
        for(int i = 0; i < clusters[tree_id].size(); i++)
            debug_clusters.at<cv::Vec3b>(clusters[tree_id][i]) = PINK_COLOR;
    }

    //Merge clusters that are close enough to the tree_cluster & find the closest cluster
    std::vector<cv::Point> tree_cluster = clusters[tree_id];
    double min_cluster_center = cluster_centers[tree_id];
    int min_cluster_id = tree_id;
    int n_merged_clusters = 0;
    int n_not_merged_clusters = 0;
    for(int c = 0; c < cluster_centers.size(); c++)
    {
        if(c!= tree_id && std::abs(cluster_centers[c] - cluster_centers[tree_id]) < thresh_depth_scaled_)
        {
            tree_cluster.reserve(tree_cluster.size() + clusters[c].size());
            tree_cluster.insert(tree_cluster.end(), clusters[c].begin(), clusters[c].end());
            if(debug_)
            {
                for(int i = 0; i < clusters[c].size(); i++)
                    if(n_merged_clusters < SHADES_OF_BLUE.size())
                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = SHADES_OF_BLUE[n_merged_clusters];
                    else
                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = cv::Vec3b(rng_.uniform(0, 255), rng_.uniform(0,255), rng_.uniform(0,255));
            }
            n_merged_clusters++;
        }
        else if (c!= tree_id)
        {
            if(debug_)
            {
                for(int i = 0; i < clusters[c].size(); i++)
                    if(n_not_merged_clusters < OTHER_COLORS.size())
                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = OTHER_COLORS[n_not_merged_clusters];
                    else
                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = cv::Vec3b(rng_.uniform(0, 255), rng_.uniform(0,255), rng_.uniform(0,255));
            }
            n_not_merged_clusters++;
            if(cluster_centers[c] < min_cluster_center)
            {
                min_cluster_center = cluster_centers[c];
                min_cluster_id = c;
            }
        }
    }

    if(min_cluster_id != tree_id)
    {
        //Merge clusters that are close enough to the clostest to complete arm cluster
        std::vector<cv::Point> arm_cluster = clusters[min_cluster_id];
        for(int c = 0; c < cluster_centers.size(); c++)
        {
            if(c!= tree_id && c!= min_cluster_id && std::abs(cluster_centers[c] - cluster_centers[min_cluster_id]) < 0.15 / depth_scale_)
            {
                arm_cluster.reserve(arm_cluster.size() + clusters[c].size());
                arm_cluster.insert(arm_cluster.end(), clusters[c].begin(), clusters[c].end());
                if(debug_)
                {
                    for(int i = 0; i < clusters[c].size(); i++)
                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = FADED_RED_COLOR;
                }
            }
        }

        //add closest cluster (should be arm cluster) to tree
        for(int i = 0; i < arm_cluster.size(); i++)
        {
            if(arm_cluster[i].y > img->image.rows/2)
            {
                tree_cluster.push_back(arm_cluster[i]);
                if(debug_)
                    debug_clusters.at<cv::Vec3b>(arm_cluster[i]) = RED_COLOR;
            }
            else
            {
                if(debug_)
                    debug_clusters.at<cv::Vec3b>(arm_cluster[i]) = FADED_RED_COLOR;
            }
        }
    }

    if(debug_)
      cv::imshow("All Clusters", debug_clusters);

    //Create binary image based on whether a point is in the merged cluster or not
    cv::Mat thresholded = cv::Mat::zeros(img->image.size(), CV_8UC1);
    for (int i = 0; i < tree_cluster.size(); i++)
    {
        //do not add border
        if(tree_cluster[i].x > 2 && tree_cluster[i].x < img->image.cols-3)
            thresholded.at<uchar>(tree_cluster[i]) = (uchar)255;
    }
    return thresholded;
}

//cv::Mat TreeCavityDetectionKmeansNodelet::findTreeBinary(std::vector<std::vector<cv::Point>> &clusters, std::vector<double> &cluster_centers, int &tree_id, const cv_bridge::CvImageConstPtr &img)
//{
//    cv::Mat debug_clusters;
//    if(debug_)
//    {
//        //Draw clusters (cluster closest to seedpoint (tree cluster) is pink, merged clusters will be shades of blue)
//        debug_clusters = cv::Mat::zeros(img->image.size(), CV_8UC3);
//        for(int i = 0; i < clusters[tree_id].size(); i++)
//            debug_clusters.at<cv::Vec3b>(clusters[tree_id][i]) = PINK_COLOR;
//    }

//    //Merge clusters that are close enough to the tree_cluster and all clusters that are in front
//    std::vector<cv::Point> tree_cluster = clusters[tree_id];
//    int n_merged_clusters = 0;
//    int n_not_merged_clusters = 0;
//    for(int c = 0; c < cluster_centers.size(); c++)
//    {
//        if(c!= tree_id && std::abs(cluster_centers[c] - cluster_centers[tree_id]) < thresh_depth_scaled_)
//        {
//            tree_cluster.reserve(tree_cluster.size() + clusters[c].size());
//            tree_cluster.insert(tree_cluster.end(), clusters[c].begin(), clusters[c].end());
//            if(debug_)
//            {
//                for(int i = 0; i < clusters[c].size(); i++)
//                    if(n_merged_clusters < SHADES_OF_BLUE.size())
//                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = SHADES_OF_BLUE[n_merged_clusters];
//                    else
//                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = cv::Vec3b(rng_.uniform(0, 255), rng_.uniform(0,255), rng_.uniform(0,255));
//            }
//            n_merged_clusters++;
//        }
//        else if (c!= tree_id && cluster_centers[c] < cluster_centers[tree_id])
//        {
//            tree_cluster.reserve(tree_cluster.size() + clusters[c].size());
//            tree_cluster.insert(tree_cluster.end(), clusters[c].begin(), clusters[c].end());
//            if(debug_)
//            {
//                for(int i = 0; i < clusters[c].size(); i++)
//                    debug_clusters.at<cv::Vec3b>(clusters[c][i]) = RED_COLOR;
//            }
//        }
//        else if (c!= tree_id)
//        {
//            if(debug_)
//            {
//                for(int i = 0; i < clusters[c].size(); i++)
//                    if(n_not_merged_clusters < OTHER_COLORS.size())
//                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = OTHER_COLORS[n_not_merged_clusters];
//                    else
//                        debug_clusters.at<cv::Vec3b>(clusters[c][i]) = cv::Vec3b(rng_.uniform(0, 255), rng_.uniform(0,255), rng_.uniform(0,255));
//            }
//            n_not_merged_clusters++;
//        }
//    }

//    if(debug_)
//        cv::imshow("All Clusters", debug_clusters);

//    //Create binary image based on whether a point is in the merged cluster or not
//    cv::Mat thresholded = cv::Mat::zeros(img->image.size(), CV_8UC1);
//    for (int i = 0; i < tree_cluster.size(); i++)
//    {
//        //do not add border
//        if(tree_cluster[i].x > 2 && tree_cluster[i].x < img->image.cols-3)
//            thresholded.at<uchar>(tree_cluster[i]) = (uchar)255;
//    }
//    return thresholded;
//}

void TreeCavityDetectionKmeansNodelet::findSeedpointContour(std::vector<cv::Point> &contour, int &id,
                     std::vector<std::vector<cv::Point> > &contours,
                     std::vector<cv::Vec4i> &hierarchy,
                     geometry_msgs::Point &seedpoint)
{
    for(int i = 0; i < contours.size(); i++)
    {
        if(cv::pointPolygonTest(contours[i], cv::Point2f(seedpoint.x,seedpoint.y),false) > 0 && hierarchy[i][3] < 0)
        {
            contour = contours[i];
            id = i;
            break;
        }
    }
}

std::pair<geometry_msgs::Point, geometry_msgs::Point> TreeCavityDetectionKmeansNodelet::minmaxTreePoint(const cv_bridge::CvImageConstPtr& img)
{
    auto mmx = std::minmax_element(prev_tree_contour_.begin(), prev_tree_contour_.end(), less_by_x);
    int min_img_tree_x = mmx.first->x;
    int max_img_tree_x = mmx.second->x;
    auto mmy = std::minmax_element(prev_tree_contour_.begin(), prev_tree_contour_.end(), less_by_y);
    int min_img_tree_y = mmy.first->y;
    int max_img_tree_y = mmy.second->y;

    double avg_depth = prev_seedpoint_depth_*depth_scale_;

    double min_x = cvtPixelCoordToWorld(min_img_tree_x, avg_depth, depth_cx_, depth_fxinv_);
    double max_x = cvtPixelCoordToWorld(max_img_tree_x, avg_depth, depth_cx_, depth_fxinv_);
    double min_y = cvtPixelCoordToWorld(min_img_tree_y, avg_depth, depth_cy_, depth_fyinv_);
    double max_y = cvtPixelCoordToWorld(max_img_tree_y, avg_depth, depth_cy_, depth_fyinv_);

    geometry_msgs::Point min_point;
    min_point.x = min_x;
    min_point.y = min_y;
    min_point.z = avg_depth;
    geometry_msgs::Point max_point;
    max_point.x = max_x;
    max_point.y = max_y;
    max_point.z = avg_depth;

    return std::make_pair(min_point, max_point);
}

std::vector<cv::RotatedRect> TreeCavityDetectionKmeansNodelet::findCavities(int id, const std::vector<std::vector<cv::Point> > &contours,
                                          const std::vector<cv::Vec4i> &hierarchy, double avg_tree_depth)
{
    //calculate min cavity width / height in pixels
    int min_width = cvtWorldDistToPixel(min_cavity_width_, avg_tree_depth, depth_fxinv_);
    int min_height = cvtWorldDistToPixel(min_cavity_height_, avg_tree_depth, depth_fyinv_);

    // Fit ellipse for each children contour of tree contour
    std::vector<cv::RotatedRect> cavities;
    for(int i = 0; i < contours.size(); i++ )
    {
        //check if the parent is the tree contour and the contour isn't super small
        if(hierarchy[i][3] == id && contours[i].size() > 4)
        {
            cv::RotatedRect ellipse;
            ellipse = cv::fitEllipse(cv::Mat(contours[i]));
            //if the fitted ellipse is larger than min cavity size and not too close to the border, add it to cavities
            if(ellipse.size.width >= min_width && ellipse.size.height >= min_height && ellipse.size.width <= 1.5*ellipse.size.height
               && cv::pointPolygonTest(contours[id], ellipse.boundingRect().tl(),true) > 4 && cv::pointPolygonTest(contours[id], ellipse.boundingRect().br(),true) > 4)
            {
                cavities.push_back(ellipse);
                if(debug_)
                {
                    std::cout << "added, ";
                }
            }
            else if(debug_)
            {
                if(ellipse.size.width < min_width)
                    std::cout << "width too small, ";
                else if(ellipse.size.height < min_height)
                    std::cout << "height too small, ";
                else if(ellipse.size.width > 1.5*ellipse.size.height)
                    std::cout << "width much larger than height, ";
                else if(cv::pointPolygonTest(contours[id], ellipse.boundingRect().tl(),true)  <= 4 || cv::pointPolygonTest(contours[id], ellipse.boundingRect().br(),true)  <= 4)
                    std::cout << "too close, ";
            }
        }
    }
    if(debug_)
    {
        std::cout << std::endl;
    }
    //sort by area
    std::sort(cavities.begin(), cavities.end(), larger_by_ellipse_area);

    return cavities;
}

void TreeCavityDetectionKmeansNodelet::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    ++count_time_;
    boost::chrono::high_resolution_clock::time_point t_start_all = boost::chrono::high_resolution_clock::now();

    updateStats();
    start_time_ = boost::posix_time::microsec_clock::local_time();

    //convert ROS image message to a CvImage.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        NODELET_ERROR("cv_bridge input exception: %s", e.what());
        return;
    }

    rci_comm::Tree tree_output;
    geometry_msgs::PointStamped cavity_output;
    cavity_output.point.x = 0;
    cavity_output.point.y = 0;
    cavity_output.point.z = 0;

    //std::cout << (double)cv_ptr->image.at<unsigned short>(85,112) << " , " << (double)cv_ptr->image.at<unsigned short>(85,112)*depth_scale_ << std::endl;

    cv::Mat cvframe;
    cv_ptr->image.convertTo(cvframe, CV_8UC1, 255.0/sensor_depth_max_scaled_);
    cv::cvtColor(cvframe, cvframe, CV_GRAY2BGR);

    NODELET_DEBUG_STREAM_ONCE("Type of ros msg: " << msg->encoding
                             << " and type of cv image: " << cvImageType(cv_ptr->image)
                             << " and after conversion: " << cvImageType(cvframe));
    NODELET_WARN_STREAM_ONCE("Size of image: width=" << cvframe.cols << " ; height=" << cvframe.rows);

    //If we have a seedpoint and haven't lost the tree, do tree detection
    if (cv_ptr->image.cols > seedpoint_.x  && cv_ptr->image.rows > seedpoint_.y
            && seedpoint_.x > 0 && seedpoint_.y > 0 && n_skipped_ < 60)
    {      
        bool found_seedpoint = true;
        if(!first_run_)
            found_seedpoint = findNewSeedPoint(cv_ptr);

        std::vector<cv::RotatedRect> cavity_ellipses; //all the potential cavities
        if(found_seedpoint)
        {
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
            int tree_id;
            std::vector<cv::Point> tree_contour;

            bool found_tree = findTree(cv_ptr, tree_id, tree_contour, contours, hierarchy);

            if(!found_tree) // we didn't find a tree contour
            {
                if(first_run_)
                    n_skipped_ = 30;
                else
                    n_skipped_++;
                NODELET_INFO_STREAM("SKIPPING! No tree contour.");
            }
            else
            {
                if (first_run_)
                {
                    prev_tree_contour_ = tree_contour;
                    prev_seedpoint_depth_ = (double) cv_ptr->image.at<unsigned short>(seedpoint_.y,seedpoint_.x);
                    std::pair<geometry_msgs::Point, geometry_msgs::Point> mm = minmaxTreePoint(cv_ptr);
                    prev_min_tree_point_ = mm.first;
                    prev_max_tree_point_ = mm.second;
                    cavity_ellipses = findCavities(tree_id, contours, hierarchy, prev_min_tree_point_.z);
                    first_run_ = false;
                    lost_pub_.publish(false);
                }
                else
                {
                    cv::Rect new_box = cv::boundingRect(tree_contour);
                    cv::Rect prev_box = cv::boundingRect(prev_tree_contour_);
                    if( (new_box.width > max_scaling_*prev_box.width && new_box.height > max_scaling_*prev_box.height)
                     || (new_box.width < prev_box.width / max_scaling_ && new_box.height < prev_box.height / max_scaling_) )
                    {
                        //do nothing, width and height changed too much (too large or too small)
                        n_skipped_++;
                        NODELET_INFO_STREAM("SKIPPING! New tree contour too large or too small.");
                    }
                    else
                    {
                        prev_tree_contour_ = tree_contour;
                        prev_seedpoint_depth_ = (double) cv_ptr->image.at<unsigned short>(seedpoint_.y,seedpoint_.x);
                        std::pair<geometry_msgs::Point, geometry_msgs::Point> mm = minmaxTreePoint(cv_ptr);
                        prev_min_tree_point_ = mm.first;
                        prev_max_tree_point_ = mm.second;
                        cavity_ellipses = findCavities(tree_id, contours, hierarchy, prev_min_tree_point_.z);
                        n_skipped_ = 0;
                    }
                }
            }
        }

        if(n_skipped_ < 30)
        {
            std::vector<std::vector<cv::Point> > tree_contours;
            tree_contours.push_back(prev_tree_contour_);
            //create a copy of the original:
            cv::Mat overlay = cvframe.clone();
            cv::drawContours(overlay, tree_contours, 0, CV_RGB(0,0,255), -1);
            //draw all found cavities
            for( int i = 0; i< cavity_ellipses.size(); i++ )
            {
                cv::Scalar color;
                if(i+2 < COLORS.size())
                    color = COLORS[i+2];
                else
                    color = cv::Scalar( rng_.uniform(0, 255), rng_.uniform(0,255), rng_.uniform(0,255) );
                cv::ellipse(overlay, cavity_ellipses[i], color, -1);
            }
            cv::Mat final_image = cv::Mat::zeros(cvframe.size(), CV_8UC3);
            //blend with the original:
            double opacity = 0.4;
            cv::addWeighted(overlay, opacity, cvframe, 1-opacity, 0, final_image);
            //cv::circle(final_image, cv::Point(seedpoint_.x, seedpoint_.y), 2, CV_RGB(255,0,0));

            //add only largest cavity to output and draw its center
            if(cavity_ellipses.size() > 0)
            {
                cv::circle(final_image, cavity_ellipses[0].center, 2, CV_RGB(255,0,0),-1);
                geometry_msgs::Point center;
                center.z = prev_min_tree_point_.z;
                center.x = cvtPixelCoordToWorld(cavity_ellipses[0].center.x, center.z, depth_cx_, depth_fxinv_);
                center.y = cvtPixelCoordToWorld(cavity_ellipses[0].center.y, center.z, depth_cy_, depth_fyinv_);
                cavity_output.point = center;
            }
            cvframe = final_image;
        }
        else
        {
            NODELET_WARN_STREAM("!!!!!!!!!!!!! LOST !!!!!!!!!!!!!!");
            lost_pub_.publish(true);
        }
    }

    //cv::circle(cvframe, cv::Point(112,85), 3, CV_RGB(0,255,0),-1);

    cavity_output.header = msg->header;
    //publish cavity
    cavity_pub_.publish(cavity_output);
    tree_output.header = msg->header;
    tree_output.tree_min = prev_min_tree_point_;
    tree_output.tree_max = prev_max_tree_point_;
    //publish tree
    tree_pub_.publish(tree_output);

    //convert opencv image to ros image and publish it
    cv_bridge::CvImagePtr out_msg_ptr(new cv_bridge::CvImage);
    out_msg_ptr->header = msg->header;
    out_msg_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    out_msg_ptr->image = cvframe;
    image_pub_.publish(out_msg_ptr->toImageMsg());

    NODELET_DEBUG_STREAM_THROTTLE(5,"The img tree detection runs at an avg of " << avg_frequency_ << " Hz");

    boost::chrono::high_resolution_clock::time_point t_end_all = boost::chrono::high_resolution_clock::now();
    time_all_ += boost::chrono::duration_cast<boost::chrono::nanoseconds>(t_end_all-t_start_all);

    if(count_time_ == 500)
    {
        boost::chrono::nanoseconds avg_time_all = time_all_ / 500;
        NODELET_DEBUG_STREAM("Time tree detection : all=" << avg_time_all << " (" << (int)(1.0 / (avg_time_all.count()*1e-9)) << " Hz)");
        time_all_ = boost::chrono::nanoseconds(0);
        count_time_ = 0;
    }
}

double TreeCavityDetectionKmeansNodelet::cvtPixelCoordToWorld(int pixel, double depth, double c, double finv)
{
    double coord = ( pixel - c ) * depth * finv;
    return coord;
}

int TreeCavityDetectionKmeansNodelet::cvtWorldDistToPixel(double world_dist, double depth, double finv)
{
    int dist = world_dist / (depth*finv);
    return dist;
}

std::string TreeCavityDetectionKmeansNodelet::cvImageType(cv::Mat cvframe)
{
    std::string r;
    uchar depth = cvframe.type() & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (cvframe.type() >> CV_CN_SHIFT);
    switch ( depth )
    {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }
    r += "C";
    r += (chans+'0');
    return r;
}

void TreeCavityDetectionKmeansNodelet::updateStats()
{
    boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration elapsed_time = end_time - start_time_;
    if(elapsed_time.total_milliseconds() > 0)
        frequency_.push_back((int)1000/elapsed_time.total_milliseconds());

    // Maximum size of queue is 100
    if (frequency_.size() == 100)
    {
        int sum = std::accumulate(frequency_.begin(),frequency_.end(),0);
        avg_frequency_ = (int) sum / 100;
        frequency_.clear();
    }
}

} //namespace rci_img_tree_detection

PLUGINLIB_DECLARE_CLASS(rci_img_tree_detection, TreeCavityDetectionKmeansNodelet, rci_img_tree_detection::TreeCavityDetectionKmeansNodelet, nodelet::Nodelet);



