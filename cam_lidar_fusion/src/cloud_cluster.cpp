#include "cloud_cluster.h"

CloudCluster::CloudCluster(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
    readParam();
    image_sub_.subscribe(nh_, image_topic, 20);
    lidar_sub_.subscribe(nh_, lidar_topic, 50);
    bd_box_sub_.subscribe(nh_, bbox_topic, 20);
    sync.reset(new Sync(enhanced_yolo_policy(10), image_sub_, lidar_sub_, bd_box_sub_));
    sync->registerCallback(boost::bind(&CloudCluster::callback, this, _1, _2, _3));
    area_thres_["person"] = pair<double, double>(1.8, 0.5);
    area_thres_["car"] = pair<double, double>(2.7, 1.8);
    area_thres_["truck"] = pair<double, double>(2.7, 1.8);
    detect_result_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detect_result_cloud", 1);
    enhanced_yolo_img_pub_ = nh_.advertise<sensor_msgs::Image>("enhanced_yolo_img", 1);
    bounding_box_pub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("bounding_boxes_lidar", 1);
    cv::namedWindow("image");
}

void CloudCluster::readParam() {
    vector<double> cam_intrinsic_data;
    vector<double> velo_to_cam_data;
    nh_local_.getParam("velo_to_cam", velo_to_cam_data);
    nh_local_.getParam("cam_intrinsic", cam_intrinsic_data);
    nh_local_.getParam("image_topic", image_topic);
    nh_local_.getParam("lidar_topic", lidar_topic);
    nh_local_.getParam("bbox_topic", bbox_topic);

    ROS_ASSERT(cam_intrinsic_data.size() == 9);
    ROS_ASSERT(velo_to_cam_data.size() == 16);
    for (int i = 0; i < cam_intrinsic_data.size(); i++) {
        cam_intrinsic_(i) = cam_intrinsic_data[i];
    }
    // the order in the eigen matrix is different from the vector
    cam_intrinsic_.transposeInPlace();
    cout << "the cam intrinsic is: " << endl << cam_intrinsic_ << endl;
    for (int i = 0; i < velo_to_cam_data.size(); i++) {
        velo_to_cam_(i) = velo_to_cam_data[i];
    }
    velo_to_cam_.transposeInPlace();
    cam_to_velo_ = velo_to_cam_.inverse();
    cout << "the velo to cam extrinsic is: " << endl << velo_to_cam_ << endl;
}

void CloudCluster::callback(const sensor_msgs::Image::ConstPtr& img_raw,
                            const sensor_msgs::PointCloud2::ConstPtr& cropped_cloud,
                            const yolo_ros::BoundingBoxes::ConstPtr& bd_boxes) {
    cout << "jump into callback" << endl;
    this->cloud_heander_ = cropped_cloud->header;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cropped_cloud, *cloud_in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *transformed_cloud, velo_to_cam_);
    // lidar coordinate(forward x+, left y+, up z+)
    // camera coordiante(right x+, down y+, forward z+) (3D-3D)
    // using the extrinsic matrix between this two coordinate system
    vector<cv::Point3d> lidar_points;
    vector<cv::Point2d> image_points;
    Eigen::Vector3d tmp_point;
    Eigen::Vector3d img_point;
    for (int i = 0; i < transformed_cloud->points.size(); i++) {
        if (transformed_cloud->points[i].z > 0) {
            lidar_points.push_back(cv::Point3d(transformed_cloud->points[i].x, transformed_cloud->points[i].y,
                                               transformed_cloud->points[i].z));
            tmp_point << transformed_cloud->points[i].x, transformed_cloud->points[i].y, transformed_cloud->points[i].z;
            tmp_point = tmp_point / tmp_point(2);
            img_point = cam_intrinsic_ * tmp_point;
            image_points.push_back(cv::Point2d(img_point(0), img_point(1)));
        }
    }
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;
    cloud_vector.resize(bd_boxes->bounding_boxes.size());
    for (int num = 0; num < bd_boxes->bounding_boxes.size(); num++) {
        cloud_vector[num] = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    }
    vector<yolo_ros::BoundingBox> box_vector = bd_boxes->bounding_boxes;

    // opencv to show projected img has bug
    cv::Mat img_to_show;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_raw);
    cv::Mat src_img = cv_ptr->image;
    if (src_img.channels() == 1) cvtColor(src_img, src_img, CV_GRAY2BGR); // very important

    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < lidar_points.size(); i++) {
        for (int j = 0; j < box_vector.size(); j++) {
            if (image_points[i].x < box_vector[j].xmax && image_points[i].x > box_vector[j].xmin &&
                image_points[i].y < box_vector[j].ymax && image_points[i].y > box_vector[j].ymin) {
                point.x = lidar_points[i].x;
                point.y = lidar_points[i].y;
                point.z = lidar_points[i].z;
                cloud_vector[j]->push_back(point);
                cv::circle(src_img, image_points[i], 1, cv::Scalar(color[j][2], color[j][1], color[j][0]));
            } else {
                continue;
            }
        }
    }
    cv::imshow("image", src_img);
    cv::waitKey(3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_lidar(new pcl::PointCloud<pcl::PointXYZ>);
    Detected_object obj_info;
    vector<Detected_object> obj_vec;
    for (int i = 0; i < cloud_vector.size(); i++) {
        // cout << "there are " << cloud_vector[i]->size() << " points in object" << i << endl;
        pcl::transformPointCloud(*cloud_vector[i], *cloud_in_lidar, cam_to_velo_);
        *out_cloud = *out_cloud + *cloud_in_lidar;
        bool success_flag = this->kd_cluster(cloud_in_lidar, box_vector[i].Class, obj_info);
        if (success_flag) {
            obj_vec.push_back(obj_info);
        }
    }
    if (not obj_vec.empty()) {
        cout << "clustered " << obj_vec.size() << " objects" << endl;
        this->drawCube(obj_vec);
    }

    sensor_msgs::PointCloud2 out_cloud_ros;
    pcl::toROSMsg(*out_cloud, out_cloud_ros);
    out_cloud_ros.header = cloud_heander_;
    detect_result_cloud_pub_.publish(out_cloud_ros);
}

// vector<cv::Point3d> CloudCluster::clusterPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in) {}
bool CloudCluster::kd_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, string category_, Detected_object& obj_info_) {
    if (in_pc->points.size() < 10) return false;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_in);
    tree->setInputCloud(cloud_in);
    std::vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid; //采用欧式聚类，以后有改进
    euclid.setInputCloud(cloud_in);
    euclid.setClusterTolerance(1);
    euclid.setMaxClusterSize(700);
    euclid.setMinClusterSize(20);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);
    // cout << "cluster " << local_indices.size() << " objects" << endl;
    obj_info_.category_ = category_;
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    if (local_indices.size() == 0) return false;
    for (auto pit = local_indices[0].indices.begin(); pit != local_indices[0].indices.end(); ++pit) {
        pcl::PointXYZ p;
        p.x = in_pc->points[*pit].x;
        p.y = in_pc->points[*pit].y;
        p.z = in_pc->points[*pit].z;

        obj_info_.centroid_.x += p.x;
        obj_info_.centroid_.y += p.y;
        obj_info_.centroid_.z += p.z;

        if (p.x < min_x) min_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.z < min_z) min_z = p.z;
        if (p.x > max_x) max_x = p.x;
        if (p.y > max_y) max_y = p.y;
        if (p.z > max_z) max_z = p.z;
    }
    // min, max points
    obj_info_.min_point_.x = min_x;
    obj_info_.min_point_.y = min_y;
    obj_info_.min_point_.z = min_z;

    obj_info_.max_point_.x = max_x;
    obj_info_.max_point_.y = max_y;
    obj_info_.max_point_.z = max_z;
    // calculate centroid, average
    if (local_indices[0].indices.size() > 0) // 求出点云簇形心
    {
        obj_info_.centroid_.x /= local_indices[0].indices.size();
        obj_info_.centroid_.y /= local_indices[0].indices.size();
        obj_info_.centroid_.z /= local_indices[0].indices.size();
    }
    //计算点云簇的长宽高
    double length_ = obj_info_.max_point_.x - obj_info_.min_point_.x;
    double width_ = obj_info_.max_point_.y - obj_info_.min_point_.y;
    double height_ = obj_info_.max_point_.z - obj_info_.min_point_.z;
    cout << "the length and width and height are: " << length_ << " " << width_ << " " << height_ << endl;
    obj_info_.bounding_box_.header = cloud_heander_;
    obj_info_.bounding_box_.pose.position.x = obj_info_.min_point_.x + length_ / 2;
    obj_info_.bounding_box_.pose.position.y = obj_info_.min_point_.y + width_ / 2;
    obj_info_.bounding_box_.pose.position.z = obj_info_.min_point_.z + height_ / 2;

    obj_info_.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
    obj_info_.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
    obj_info_.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);
    return true;
}
//发送聚类得到的3Dboundingbox
void CloudCluster::drawCube(vector<Detected_object>& obj_vec_) {
    jsk_recognition_msgs::BoundingBoxArray bbox_array; // box矩阵
    bbox_array.header = cloud_heander_;
    for (size_t i = 0; i < obj_vec_.size(); i++) {
        bbox_array.boxes.push_back(obj_vec_[i].bounding_box_);
    }
    bounding_box_pub_.publish(bbox_array);
}

void callback1(const yolo_ros::ObjectCountConstPtr& objct_num) { cout << objct_num->count << endl; }
int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_cluster");
    ros::NodeHandle nh(""), nh_local("");
    CloudCluster cloud_cluster(nh, nh_local);
    ros::spin();
    return 0;
}
