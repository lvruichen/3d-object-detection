#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
void call_back(const sensor_msgs::Image::ConstPtr& img_in) {
    cv::Mat img_to_show;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::BGR8);
    img_to_show = cv_ptr->image;
    cv::circle(img_to_show, cv::Point2d(50, 50), 3, cv::Scalar(255, 0, 0), 3);
    cv::namedWindow("image");
    cv::imshow("image", img_to_show);
    cv::waitKey(3);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "image_shower_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/image_raw", 1, call_back);
    ros::spin();
    return 0;
}
