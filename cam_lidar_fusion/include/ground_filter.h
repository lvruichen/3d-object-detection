/*
 * Note: Remove ground from the points cloud.
 *      Algrothm proposed by
 *     《Fast segmentation of 3D point clouds: A paradigm on LiDAR data for
 * autonomous vehicle applications》
 */
#ifndef GROUND_FILTER_H_
#define GROUND_FILTER_H_
#include <omp.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <Eigen/Core>
#include <Eigen/Dense>
class GroundPlaneFilter
{
public:
    /** \brief constructor
     */
    GroundPlaneFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~GroundPlaneFilter();

private:
    typedef pcl::console::TicToc TicToc;

    void updateParams();

    void callBack(const sensor_msgs::PointCloud2ConstPtr fused_cloud);

    void extractInitialSeeds(pcl::PointCloud<pcl::PointXYZI>& in_cloud);

    void estimatePlane();

    inline void allClear()
    {
    }

    void paperMethod(pcl::PointCloud<pcl::PointXYZI>& in_cloud);

    void backupMethod(pcl::PointCloud<pcl::PointXYZI>& in_cloud,
                      double ground_height);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::Subscriber sub_;
    ros::Publisher point_no_ground_pub_;
    ros::Publisher point_ground_pub_;
    ros::Publisher ground_height_pub_;
    Eigen::Vector3f norm_vector_;  // norm vector that describes the plane
    double d_;                     // d that describe the plane. n'x + d = 0
    pcl::PointCloud<pcl::PointXYZI> point_ground_;
    pcl::PointCloud<pcl::PointXYZI> point_no_ground_;
    TicToc tt_;
    std::string topic_str_;
    int n_iter_;          // number of iteration
    int n_lpr_;           // number of points used to estimate the LPR
    double thres_seeds_;  // threshold for points to be considered initial
                          // seeds.
    double thres_dist_;   // threshold distance from the plane
};

#endif