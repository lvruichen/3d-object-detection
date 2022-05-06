#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
int main(int argc, char **argv) {
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/image_raw", 1);
    std::string config_path = "/home/eric/a_ros_ws/object_dection_ws/src/3d-object-detection/cam_lidar_fusion/config";
    std::string file_name = "image_lidar.yaml";
    std::string config_file_name = config_path + "/" + file_name;
    cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);
    std::string image_path;
    fs_reader["image_path"] >> image_path;
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_ANYCOLOR);
    cv::imshow("image", image);
    cv::waitKey(0);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    ros::Rate loop_rate(5);
    while (ros::ok()) {
        msg->header.stamp = ros::Time::now();
        pub.publish(msg);
        ros::spinOnce();
        // ROS_INFO("send once");
        loop_rate.sleep();
    }

    return 0;
}