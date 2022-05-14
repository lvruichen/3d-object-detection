#ifndef CLOUD_CLUSTER_H_
#define CLOUD_CLUSTER_H_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/search/impl/search.hpp>

#include "yolo_ros/BoundingBox.h"
#include "yolo_ros/BoundingBoxes.h"
#include "yolo_ros/ObjectCount.h"
using namespace std;

class CloudCluster {
public:
    CloudCluster(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~CloudCluster(){};
    struct Detected_object {
        pcl::PointXYZ min_point_;
        pcl::PointXYZ max_point_;
        pcl::PointXYZ centroid_;
        string category_;
        jsk_recognition_msgs::BoundingBox bounding_box_;
    };

private:
    void readParam();
    void callback(const sensor_msgs::ImageConstPtr& img_raw, const sensor_msgs::PointCloud2ConstPtr& cropped_cloud,
                  const yolo_ros::BoundingBoxesConstPtr& bd_boxes);
    bool kd_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, string category_, Detected_object& obj_info_);
    void drawCube(vector<Detected_object>& obj_vec_);
    // bool filterBboxByArea();//以后有时间再改进
    ros::NodeHandle nh_, nh_local_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    message_filters::Subscriber<yolo_ros::BoundingBoxes> bd_box_sub_;

    ros::Publisher detect_result_cloud_pub_;
    ros::Publisher enhanced_yolo_img_pub_;
    ros::Publisher bounding_box_pub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2,
                                                            yolo_ros::BoundingBoxes>
        enhanced_yolo_policy;
    typedef message_filters::Synchronizer<enhanced_yolo_policy> Sync;
    boost::shared_ptr<Sync> sync;

    Eigen::Matrix3d cam_intrinsic_;
    Eigen::Matrix4d velo_to_cam_; // look in camera frame
    Eigen::Matrix4d cam_to_velo_; // look in lidar frame
    string image_topic;
    string lidar_topic;
    string bbox_topic;
    std_msgs::Header cloud_heander_;
    map<string, pair<double, double>> area_thres_;
    int color[21][3] = {{255, 0, 0},     {255, 69, 0},    {255, 99, 71},   {255, 140, 0},   {255, 165, 0},
                        {238, 173, 14},  {255, 193, 37},  {255, 255, 0},   {255, 236, 139}, {202, 255, 112},
                        {0, 255, 0},     {84, 255, 159},  {127, 255, 212}, {0, 229, 238},   {152, 245, 255},
                        {178, 223, 238}, {126, 192, 238}, {28, 134, 238},  {0, 0, 255},     {72, 118, 255},
                        {122, 103, 238}};
};

#endif