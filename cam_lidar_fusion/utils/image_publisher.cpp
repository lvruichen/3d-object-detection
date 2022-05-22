#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
int main(int argc, char **argv) {
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/kitti/camera_color_left/image_raw", 1);
    std::string config_path = "/home/eric/a_ros_ws/object_detection_ws/src/3d-object-detection/cam_lidar_fusion/config";
    std::string file_name = "image_lidar.yaml";
    std::string config_file_name = config_path + "/" + file_name;
    cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);
    std::string image_path;
    fs_reader["image_path"] >> image_path;
    cv::Mat image = cv::imread(image_path);
    cv::namedWindow("image");
    cv::imshow("image", image);
    cv::waitKey(1000);
    cv::destroyWindow("image");
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "camera_link";
        pub.publish(msg);
        ros::spinOnce();
        // ROS_INFO("send once");
        loop_rate.sleep();
    }

    return 0;
}
