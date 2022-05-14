
#include "ground_filter.h"

using namespace std;

GroundPlaneFilter::GroundPlaneFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
    updateParams();
    sub_ = nh_.subscribe(topic_str_, 1, &GroundPlaneFilter::callBack, this);
    point_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_ground", 10);
    point_no_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_no_ground", 10);
    ground_height_pub_ = nh_.advertise<std_msgs::Float32>("ground_height", 10);
}

GroundPlaneFilter::~GroundPlaneFilter() {
    nh_local_.deleteParam("n_iter");
    nh_local_.deleteParam("n_lpr");
    nh_local_.deleteParam("thres_seeds");
    nh_local_.deleteParam("thres_dist");
    nh_local_.deleteParam("topic_str");
}

void GroundPlaneFilter::updateParams() {
    nh_local_.param<int>("n_iter", n_iter_, 3);
    nh_local_.param<int>("n_lpr", n_lpr_, 100);
    nh_local_.param<double>("thres_seeds", thres_seeds_, 0.5);
    nh_local_.param<double>("thres_dist", thres_dist_, 0.3);
    nh_local_.param<string>("topic_str", topic_str_, "/lidar_points");

    assert(n_lpr_ > 0);
}

void GroundPlaneFilter::callBack(const sensor_msgs::PointCloud2ConstPtr fused_cloud) {
    pcl::PointCloud<pcl::PointXYZI> in_cloud;
    if (fused_cloud->fields.at(3).name != "intensity") {
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        pcl::fromROSMsg(*fused_cloud, cloud_xyz);
        pcl::copyPointCloud(cloud_xyz, in_cloud);
    } else {
        pcl::fromROSMsg(*fused_cloud, in_cloud);
    }

    in_cloud.header.frame_id = fused_cloud->header.frame_id;
    in_cloud.header.stamp = fused_cloud->header.stamp.nsec;

    pcl::PointCloud<pcl::PointXYZI> in_valid_cloud;
    for (int i = 0; i < in_cloud.size(); i++) {
        if (pcl_isfinite(in_cloud[i].x)) in_valid_cloud.push_back(in_cloud[i]);
    }
    allClear();
    paperMethod(in_valid_cloud);

    sensor_msgs::PointCloud2 ros_point_no_ground;
    pcl::toROSMsg(point_no_ground_, ros_point_no_ground);
    ros_point_no_ground.header.stamp = fused_cloud->header.stamp;
    ros_point_no_ground.header.frame_id = fused_cloud->header.frame_id;
    point_no_ground_pub_.publish(ros_point_no_ground);
}

void GroundPlaneFilter::extractInitialSeeds(pcl::PointCloud<pcl::PointXYZI>& in_cloud) {
    sort(in_cloud.points.begin(), in_cloud.points.end(),
         [](const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs) { return lhs.z < rhs.z; });
    // get mean z value of LPR
    double LPR_height = 0; // Lowest Point Representation
    for (int i = 0; i < n_lpr_; i++) {
        LPR_height += in_cloud.points[i].z;
    }
    LPR_height /= n_lpr_;

    for (int i = 0; i < in_cloud.points.size(); i++) {
        if (isnan(in_cloud.points[i].x) || isnan(in_cloud.points[i].y || isnan(in_cloud.points[i].z))) {
            continue;
        }

        if (in_cloud.points[i].z < LPR_height + thres_seeds_) {
            point_ground_.push_back(in_cloud.points[i]);
        }
    }
}

void GroundPlaneFilter::estimatePlane() {
    Eigen::Matrix3f cov;
    Eigen::Vector4f mean;
    pcl::computeMeanAndCovarianceMatrix(point_ground_, cov, mean);
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    norm_vector_ = svd.matrixU().col(2);
    auto mean_point = mean.head<3>();
    d_ = -norm_vector_.transpose() * mean_point;
    std_msgs::Float32 ground_height_msg;

    ground_height_msg.data = d_;
    ground_height_pub_.publish(ground_height_msg);
}

void GroundPlaneFilter::paperMethod(pcl::PointCloud<pcl::PointXYZI>& in_cloud) {
    tt_.tic();
    extractInitialSeeds(in_cloud);
    // cout << "Elapsed: " << tt_.toc() << "ms" << endl;
    for (int i = 0; i < n_iter_; i++) {
        estimatePlane();
        point_ground_.clear();
        point_no_ground_.clear();
        Eigen::Vector3f pt_vec;
        for (auto pt : in_cloud.points) {
            if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z)) continue;
            pt_vec.x() = pt.x;
            pt_vec.y() = pt.y;
            pt_vec.z() = pt.z;
            if (norm_vector_.transpose() * pt_vec + d_ > thres_dist_)
                point_no_ground_.points.push_back(pt);
            else
                point_ground_.points.push_back(pt);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grounf_filter_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    GroundPlaneFilter gpf(nh, nh_local);
    ros::spin();
    return 0;
}
