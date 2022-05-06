#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointI> PointCloudI;

void read_from_bin(PointCloudT &pc_, string file_path) {
    pc_.clear();
    fstream input(file_path.c_str(), ios::in | ios::binary);
    if (!input.good()) {
        cerr << "Couldn't read file: " << file_path << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);
    float inten;
    for (int i = 0; input.good() && !input.eof(); i++) {
        PointT point;
        input.read((char *)&point.x, 3 * sizeof(float));
        input.read((char *)&inten, sizeof(float));
        pc_.push_back(point);
    }
    input.close();
}

void read_from_bin(PointCloudI &pc_, string file_path) {
    pc_.clear();
    fstream input(file_path.c_str(), ios::in | ios::binary);
    if (!input.good()) {
        cerr << "Couldn't read file: " << file_path << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);
    float inten;
    for (int i = 0; input.good() && !input.eof(); i++) {
        PointI point;
        input.read((char *)&point.x, 4 * sizeof(float));
        pc_.push_back(point);
    }
    input.close();
}
void read_from_pcd() {}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_publisher_node");
    ros::NodeHandle nh;
    YAML::Node config = YAML::LoadFile(
        "/home/eric/a_ros_ws/object_dection_ws/src/3d-object-detection/cam_lidar_fusion/config/image_lidar.yaml");
    cout << "lidar_path: " << config["lidar_path"].as<string>() << endl;
    string lidar_path = config["lidar_path"].as<string>();
    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("lidar_points", 1);
    PointCloudT point_cloud;
    PointCloudI point_cloud_i;
    sensor_msgs::PointCloud2 velodyne_points;
    string suffixStr = lidar_path.substr(lidar_path.find_last_of('.') + 1);
    cout << "the suffix is " << suffixStr << endl;
    bool is_bin = suffixStr == "bin" ? true : false;
    if (config["pub_xyzi"]) {
        read_from_bin(point_cloud_i, lidar_path);
        pcl::toROSMsg(point_cloud_i, velodyne_points);
    } else {
        read_from_bin(point_cloud, lidar_path);
        pcl::toROSMsg(point_cloud, velodyne_points);
    }
    ros::Rate loop_rate(10);

    velodyne_points.header.frame_id = "lidar_link";
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        velodyne_points.header.stamp = now;
        lidar_pub.publish(velodyne_points);
        loop_rate.sleep();
    }

    return 0;
}
