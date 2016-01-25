#ifndef TREE_CAVITY_DETECTION_KMEANS_NODELET_H
#define TREE_CAVITY_DETECTION_KMEANS_NODELET_H

//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
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
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>

#include <rci_comm/InitSeedPoint.h>
#include <rci_comm/Tree.h>
#include <rci_comm/GetTreeDetectionParams.h>
#include <rci_comm/SetTreeDetectionParams.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>

#include <omp.h>

const std::array<cv::Scalar, 12> COLORS = { cv::Scalar(255,0,0), cv::Scalar(0,0,255),
                                            cv::Scalar(0,255,0), cv::Scalar(255,0,255),
                                            cv::Scalar(0,255,255), cv::Scalar(255,255,0),
                                            cv::Scalar(255,0,127), cv::Scalar(0,128,255),
                                            cv::Scalar(128,255,0), cv::Scalar(255,128,0),
                                            cv::Scalar(127,0,255), cv::Scalar(0,255,128)};

const cv::Vec3b PINK_COLOR(204,0,203);
const cv::Vec3b RED_COLOR(0,0,255);
const cv::Vec3b FADED_RED_COLOR(153,153,255);
const std::array<cv::Vec3b, 8> SHADES_OF_BLUE = {cv::Vec3b(153,0,0), cv::Vec3b(255,0,0),
                                                 cv::Vec3b(255,102,102), cv::Vec3b(153,76,0),
                                                 cv::Vec3b(255,128,0), cv::Vec3b(255,178,102),
                                                 cv::Vec3b(255,255,0), cv::Vec3b(255,255,102)};

const std::array<cv::Vec3b, 9> OTHER_COLORS = {cv::Vec3b(0,255,0), cv::Vec3b(0,255,255),
                                                cv::Vec3b(0,255,128), cv::Vec3b(0,128,255),
                                                cv::Vec3b(255,0,127), cv::Vec3b(128,255,0),
                                                cv::Vec3b(0,76,153), cv::Vec3b(160,160,160),
                                                cv::Vec3b(255,255,255)};

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_img_tree_detection
{

inline bool less_by_x(const cv::Point& lhs, const cv::Point& rhs)
{
  return lhs.x < rhs.x;
}


inline bool less_by_y(const cv::Point& lhs, const cv::Point& rhs)
{
  return lhs.y < rhs.y;
}

inline bool larger_by_ellipse_area (cv::RotatedRect i,cv::RotatedRect j)
{

    return ( (i.size.width*i.size.height*M_PI) > (j.size.width*j.size.height*M_PI) );
}


/*****************************************************************************
** Class
*****************************************************************************/

/*! Tree Cavity Detection Kmeans nodelet,
 * detects the tree and cavity in the depth image using the OpenCV findContour function
 */
class TreeCavityDetectionKmeansNodelet : public nodelet::Nodelet
{
public:

    /*! Constructor
     */
    TreeCavityDetectionKmeansNodelet();

private:

    /*! All initialization of the ROS infrastructure
     */
    virtual void onInit();


    /*! The callback function for the ros subscriber image_sub_,
     * where the depth image is grabed and processed for tree / cavity detection
     */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);


    /*! The callback function for the ros service seed_service_,
     * where the tree seedpoint is initialized
     */
    bool initSeedPoint(rci_comm::InitSeedPoint::Request &req, rci_comm::InitSeedPoint::Response &res);


    /*! The callback function for the ros service get_params_service_,
     * where the parameters are sent back
     */
    bool getParams(rci_comm::GetTreeDetectionParamsRequest &req, rci_comm::GetTreeDetectionParamsResponse &res);


    /*! The callback function for the ros service set_params_service_,
     * where the parameters are changed
     */
    bool setParams(rci_comm::SetTreeDetectionParamsRequest &req, rci_comm::SetTreeDetectionParamsResponse &res);


    /*! Find a new tree seedpoint in the image img
     */
    bool findNewSeedPoint(const cv_bridge::CvImageConstPtr &img);


    /*! Find a tree in the image img
     */
    bool findTree(const cv_bridge::CvImageConstPtr& img, int& id, std::vector<cv::Point>& contour,
                  std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Vec4i>& hierarchy);


    /*! Generate a center.size() number of center points from among the values of samples, for kmeans that are all different and different from seedpoint
     */
    void generateRandomNotSeedCenter(std::vector<double> &center, const std::vector<cv::Point> &samples, const cv_bridge::CvImageConstPtr &img);


    /*! Do kmeans
     */
    void kmeans(std::vector<std::vector<cv::Point>> &clusters, std::vector<double> &cluster_centers, const std::vector<cv::Point> &nonzero_locations, const cv_bridge::CvImageConstPtr &img);


    /*! Find the cluster with the center closest to the seedpoint
     */
    void findClosestSeedpointTreeCluster(std::vector<double> &cluster_centers, int &tree_id, const cv_bridge::CvImageConstPtr &img);


    /*! Merge clusters that are close enough to the tree_cluster,
    *  then create a binary image based on whether an image point is in the merged cluster or not
    */
   cv::Mat findTreeBinary(std::vector<std::vector<cv::Point>> &clusters, std::vector<double> &cluster_centers, int &tree_id, const cv_bridge::CvImageConstPtr &img);


  /*! Check the points of all the clusters that are in fron of the tree cluster and if they are in the tree contour,
  *   fill this point in the tree binary
  */
  void fillFrontClusterHoles(cv::Mat &tree_binary, std::vector<cv::Point> &tree_contour,
                             std::vector<std::vector<cv::Point>> &clusters, std::vector<double> &cluster_centers, int &tree_id);


   /*! Find the contour in contours which contains the seedpoint (and has no parent contour)
    */
   void findSeedpointContour( std::vector<cv::Point> &contour, int &id,
                              std::vector<std::vector<cv::Point> > &contours,
                              std::vector<cv::Vec4i> &hierarchy,
                              geometry_msgs::Point &seedpoint);

   /*! Return the min and max x/y coords of the tree in m and the avg z coord of the tree in m
    */
    std::pair<geometry_msgs::Point, geometry_msgs::Point> minmaxTreePoint(const cv_bridge::CvImageConstPtr& img);


    /*! Find potential cavities in the tree contour with id
     */
    std::vector<cv::RotatedRect> findCavities(int id, const std::vector<std::vector<cv::Point> > &contours,
                                              const std::vector<cv::Vec4i> &hierarchy, double avg_tree_depth);


    /*! Convert an image pixel coord to a real world coord in m, given its depth in m, the central point and the focal length
     */
    double cvtPixelCoordToWorld(int pixel_x, double depth, double c, double finv);

    /*! Convert a real world dist (horizontal or vertical) in m, given its depth in m and its focal length to an image pixel dist
     */
    int cvtWorldDistToPixel(double world_width, double depth, double finv);

    /*! Get the image type of the cv image cvframe
     */
    std::string cvImageType(cv::Mat cvframe);

    /*! Update the stats, i.e. avg_frequency
     */
    void updateStats();


    ros::NodeHandle nh_, private_nh_;

    image_transport::Subscriber image_sub_;
    ros::ServiceServer seed_service_, get_params_service_, set_params_service_;

    image_transport::Publisher image_pub_;
    ros::Publisher lost_pub_;
    ros::Publisher tree_pub_, cavity_pub_;

    int sensor_depth_max_;
    double sensor_depth_max_scaled_;
    double depth_scale_, depth_fxinv_, depth_fyinv_, depth_cx_, depth_cy_;

    double thresh_seed_, thresh_depth_, thresh_seed_scaled_, thresh_depth_scaled_;
    double max_scaling_, min_cavity_width_, min_cavity_height_;

    int k_, kmeans_max_trys_;
    double kmeans_tol_, kmeans_tol_scaled_;

    bool debug_;
    std::string sensor_name_;

    geometry_msgs::Point seedpoint_;
    double prev_seedpoint_depth_;
    std::vector<cv::Point> prev_tree_contour_;
    geometry_msgs::Point prev_min_tree_point_, prev_max_tree_point_;
    bool first_run_;
    int n_skipped_;

    boost::posix_time::ptime start_time_;
    std::vector<int> frequency_;
    int avg_frequency_;

    double min_depth_, max_depth_;

    cv::RNG rng_;

    bool is_simulated_;

    boost::chrono::nanoseconds time_all_;
    int count_time_;
};

}  // namespace rci_img_tree_detection

#endif // TREE_CAVITY_DETECTION_KMEANS_NODELET_H
